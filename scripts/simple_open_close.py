#!/usr/bin/env python3
"""Local single-axis OPEN/CLOSE demo using verified firmware-bounded steps."""

import json
import argparse
import math
import os
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import signal
import threading
import time
from urllib.request import urlopen

import rclpy
from sensor_msgs.msg import PointCloud
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray, String

from rosaia_workflow import ROOT, api, nodes, preflight

# Continuous OPEN passed; CLOSE repeatedly triggered the host-current guard.
# Disabled pending bench verification; do not remove the current protection.
CONTINUOUS_ENABLED = False

def timer_wrapped(previous, current):
    return (previous['mcu_time_us'] > 2**32-1000000
            and current['mcu_time_us'] < 1000000
            and current['sequence'] == previous['sequence']+1
            and all(abs(a-b) <= 32 for a, b in zip(previous['encoder_counts'], current['encoder_counts'])))


def validate_wrap_reference(path, current):
    previous = None
    confirmed = wrapped = False
    with path.open() as source:
        for line in source:
            event = json.loads(line)
            if event.get('type') == 'session_started':
                confirmed = bool(event.get('home_calibrated'))
            if event.get('type') != 'motor':
                continue
            if previous and event['mcu_time_us'] < previous['mcu_time_us']:
                wrapped = timer_wrapped(previous, event)
                if not wrapped:
                    confirmed = False
            previous = event
    if not (confirmed and wrapped and previous
            and current['encoder_counts'] == previous['encoder_counts']
            and 0 <= current['sequence']-previous['sequence'] < 500
            and current['mcu_time_us'] >= previous['mcu_time_us']):
        raise RuntimeError('Cannot establish encoder-reference continuity from timer-wrap log')


def scaled_pwm(base, percent):
    if type(percent) is not int or not 50 <= percent <= 120:
        raise ValueError('PWM intensity must be between 50 and 120 percent')
    return min(255, max(1, round(base*percent/100)))


def validate_target(value):
    """Only finite integer positions in the locally explored range are accepted."""
    if isinstance(value, bool):
        raise ValueError('Target must be an integer between 0 and 30')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('Target must be an integer between 0 and 30') from exc
    if not math.isfinite(numeric) or numeric != round(numeric) or not 0 <= numeric <= 30:
        raise ValueError('Target must be an integer between 0 and 30')
    return int(numeric)


def validate_nominals(open_q, close_q):
    open_q, close_q = validate_target(open_q), validate_target(close_q)
    if open_q >= close_q:
        raise ValueError('OPEN must be smaller than CLOSE')
    return open_q, close_q


def validate_jog(axis, direction, q, step_counts=5):
    """One manual click, not a queued or continuous motion."""
    if type(axis) is not int or axis not in (0, 1):
        raise ValueError('Select motor axis 0 or 1')
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError('Direction must be +1 or -1')
    if type(step_counts) is not int or not 1 <= step_counts <= 10:
        raise ValueError('Step must be an integer from 1 to 10 (firmware limit)')
    return -direction*step_counts


def smooth_pwm(remaining, elapsed, peak, lead=0., boost=False):
    """Bounded ramp and end taper; no force-control or learned-model claim."""
    if remaining <= 4:
        return 0
    ramp = min(1., max(0., elapsed/.35))
    taper = .65+.35*min(1., math.sqrt(remaining/35.))
    sync = max(.65, 1.-max(0., lead)*2.)
    if boost:
        taper = sync = 1.
    return round(peak*ramp*taper*sync)


def velocity_pwm(remaining, speed, span, elapsed, peak, idle_s, lead=0.):
    """Low-speed encoder feedback, with bounded breakaway assistance."""
    if remaining <= 4:
        return 0
    desired = min(max(30., min(100., span/1.5)), remaining*2.5)
    desired *= max(.5, 1.-max(0., lead)*2.)
    pwm = .55*peak+.4*(desired-speed)
    pwm *= min(1., max(0., elapsed/.25))
    if idle_s > .35 and speed < 5:
        pwm = max(pwm, peak*min(1., .7+(idle_s-.35)*2.))
    return round(max(0., min(peak, pwm)))


def save_nominal_file(path, values):
    """Replace saved targets atomically, with no robot interaction."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                         prefix='.nominals_', delete=False) as output:
            temporary = output.name
            json.dump(values, output, indent=2)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            os.unlink(temporary)


def next_target(q, open_q=0, close_q=20):
    """Release incomplete closes; never accumulate extra contraction on retries."""
    if not -16 <= q <= 40:
        raise ValueError('encoder outside verified local demo range')
    validate_nominals(open_q, close_q)
    return close_q if q <= open_q+4 else open_q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-from-log', type=lambda value: ROOT / value)
    options = parser.parse_args()
    state = preflight()
    if not state.get('home_calibrated'):
        if options.resume_from_log is None:
            raise RuntimeError('confirmed HOME required; this program never zeros encoders')
        validate_wrap_reference(options.resume_from_log, state['motor'])
    if '/encoder_velocity_controller' in nodes():
        raise RuntimeError('stop the velocity controller before the simple demo')
    rclpy.init()
    node = rclpy.create_node('simple_open_close')
    latest = [None, 0.0]
    shape_time = [0.0, 0.0]
    previous_motor = [state['motor']]
    cancelled = threading.Event()
    shutdown = threading.Event()
    lock = threading.RLock()
    status = {'moving': False, 'error': None, 'armed': True,
              'axis': 0, 'camera_arm': 'arm_2', 'open_q': 0, 'close_q': 20,
              'pwm_percent': 100}
    nominal_path = ROOT / 'data/control_tests/simple_demo_nominals.json'
    jog_nominal_path = ROOT / 'data/control_tests/jog_nominals.json'
    jog_nominals = json.loads(jog_nominal_path.read_text()) if jog_nominal_path.exists() else {}
    if nominal_path.exists():
        saved = json.loads(nominal_path.read_text())
        if saved.get('axis') != 0 or saved.get('camera_arm') != 'arm_2':
            raise RuntimeError('Saved nominal mapping disagrees with verified motor')
        open_q, close_q = validate_nominals(saved['open_q'], saved['close_q'])
        status.update(open_q=open_q, close_q=close_q)
    folder = ROOT / 'data/sessions' / time.strftime('simple_demo_%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    log = (folder / 'signals.jsonl').open('w')
    output_lock = threading.Lock()
    def record(value):
        with output_lock:
            log.write(json.dumps({'host_time_ns': time.time_ns(), **value})+'\n')
            log.flush()

    record({'type': 'session_started', 'initial_motor': state['motor'],
            'home_calibrated': state['home_calibrated'] or options.resume_from_log is not None,
            'resume_from_log': str(options.resume_from_log) if options.resume_from_log else None,
            'display_geometry': state.get('display_geometry'),
            'command_semantics': 'raw axes; operational q=[encoder0,-encoder1]',
            'confirmed_mapping': {'axis_0': 'camera arm_2', 'axis_1': 'unconfirmed'},
            'pwm_contraction': [150, 200], 'pwm_release': [120, 140]})

    def motor_callback(msg):
        s = json.loads(msg.data)
        if s['mcu_time_us'] < previous_motor[0]['mcu_time_us'] and not timer_wrapped(previous_motor[0], s):
            status.update(armed=False, error='Board reset: HOME must be confirmed again')
            cancelled.set()
        previous_motor[0] = s
        latest[:] = [s, time.monotonic()]
        record({'type': 'motor', **s})

    def shape_callback(msg, index):
        shape_time[index] = time.monotonic()
        record({'type': 'shape', 'frame': msg.header.frame_id,
                'points': [[p.x, p.y, p.z] for p in msg.points]})

    subscriptions = [node.create_subscription(String, '/motor_state', motor_callback, 30)]
    subscriptions += [node.create_subscription(PointCloud, f'/arm_{i+1}/aruco_state',
                      lambda msg, index=i: shape_callback(msg, index), qos_profile_sensor_data)
                      for i in (0, 1)]
    step = node.create_publisher(Int16MultiArray, '/motor_step_command', 10)
    zero = node.create_publisher(Int16MultiArray, '/motor_command', 10)

    def halt():
        for _ in range(3):
            zero.publish(Int16MultiArray(data=[0, 0]))
            time.sleep(.01)

    def checked(allow_home_limit=False):
        s = latest[0]
        if s is None or time.monotonic()-latest[1] > .2:
            raise RuntimeError('Motor telemetry unavailable')
        if any(time.monotonic()-t > .3 for t in shape_time):
            raise RuntimeError('Camera tracking unavailable')
        if not status['armed']:
            raise RuntimeError('HOME reference unavailable')
        # The firmware still blocks driving past zero. During an OPEN only,
        # its recoverable limit flag may stop one axis while the other finishes.
        allowed_flags = 16
        q = [s['encoder_counts'][0], -s['encoder_counts'][1]]
        if allow_home_limit and any(-12 <= v <= 0 for v in q) and min(q) >= -12:
            allowed_flags |= 64
        if int(s['flags']) & ~allowed_flags:
            raise RuntimeError(f'Firmware fault flags={s["flags"]}')
        if any(abs(v) > 2.8 for v in s['current_a_host']):
            raise RuntimeError('Host current estimate exceeded limit')
        return s

    def smooth_targets(pose):
        if pose not in ('open', 'close'):
            raise ValueError('Select OPEN or CLOSE')
        targets = []
        for axis in (0, 1):
            saved = jog_nominals.get(str(axis), {})
            value = saved.get(pose, {}).get('q', 0 if pose == 'open' else None)
            if type(value) is not int:
                raise ValueError('Save CLOSE for both motors first')
            targets.append(value)
        return targets

    def smooth_worker(pose, targets):
        try:
            s = checked(allow_home_limit=pose == 'open')
            start_q = [s['encoder_counts'][0], -s['encoder_counts'][1]]
            direction = [1 if targets[i] > start_q[i] else -1 for i in (0, 1)]
            # Never over-contract past the author-saved nominal to fight tendon
            # relaxation: the pilot compensation trial triggered current guard.
            approach_targets = list(targets)
            span = [abs(approach_targets[i]-start_q[i]) for i in (0, 1)]
            done = [start_q[i] <= targets[i]+4 if pose == 'open' else abs(targets[i]-start_q[i]) <= 4
                    for i in (0, 1)]
            initially_done = list(done)
            velocity = [0., 0.]
            last_q = list(start_q)
            last_motion = [time.monotonic()]*2
            last_mcu = s['mcu_time_us']
            started = time.monotonic()
            record({'type': 'smooth_started', 'pose': pose, 'initial': start_q,
                    'targets': targets, 'approach_targets': approach_targets, 'ramp_s': .25})
            while not all(done):
                if cancelled.is_set():
                    raise RuntimeError('Stopped')
                s = checked(allow_home_limit=pose == 'open')
                now = time.monotonic()
                if now-started > 8:
                    raise RuntimeError('Continuous motion deadline')
                q = [s['encoder_counts'][0], -s['encoder_counts'][1]]
                dt = (s['mcu_time_us']-last_mcu)/1e6
                if dt > 0:
                    for axis in (0, 1):
                        velocity[axis] = .25*velocity[axis]+.75*(q[axis]-last_q[axis])/dt
                        if q[axis] != last_q[axis]:
                            last_motion[axis] = now
                    last_q = list(q)
                    last_mcu = s['mcu_time_us']
                progress = [min(1., max(0., direction[i]*(q[i]-start_q[i])/max(span[i], 1)))
                            for i in (0, 1)]
                commands = [0, 0]
                for axis in (0, 1):
                    if initially_done[axis]:
                        if abs(q[axis]-start_q[axis]) > 16:
                            raise RuntimeError('Unexpected passive-axis displacement')
                        continue
                    if direction[axis]*(q[axis]-start_q[axis]) < -4:
                        raise RuntimeError('Unexpected continuous motion direction')
                    if direction[axis]*(q[axis]-targets[axis]) > 25:
                        raise RuntimeError('Continuous target overshoot guard')
                    remaining = direction[axis]*(approach_targets[axis]-q[axis])
                    # Anticipate stopping/coasting using measured encoder speed.
                    braking = 0. if pose == 'close' and axis == 1 else min(
                        12., max(0., direction[axis]*velocity[axis])*.04)
                    if remaining <= 4+braking:
                        done[axis] = True
                    if done[axis]:
                        continue
                    if now-last_motion[axis] > 1.2:
                        raise RuntimeError(f'Motor {axis}: no encoder motion')
                    peak = (150, 200)[axis] if direction[axis] > 0 else (120, 140)[axis]
                    pwm = velocity_pwm(remaining, direction[axis]*velocity[axis],
                                       span[axis], now-started, peak,
                                       now-last_motion[axis], progress[axis]-progress[1-axis])
                    commands[axis] = -direction[axis]*pwm
                zero.publish(Int16MultiArray(data=commands))
                record({'type': 'smooth_command', 'pose': pose, 'q': q,
                        'targets': targets, 'commands': commands,
                        'velocity_estimate': list(velocity)})
                if cancelled.wait(.02):
                    raise RuntimeError('Stopped')
            halt()
            end = time.monotonic()+.7
            while time.monotonic() < end:
                if cancelled.wait(.02):
                    raise RuntimeError('Stopped')
                checked(allow_home_limit=pose == 'open')
            s = checked(allow_home_limit=pose == 'open')
            actual = [s['encoder_counts'][0], -s['encoder_counts'][1]]
            errors = [actual[i]-targets[i] for i in (0, 1)]
            status['smooth_result'] = {'pose': pose, 'targets': targets,
                                       'actual': actual, 'errors': errors}
            record({'type': 'smooth_finished', **status['smooth_result']})
            if any(abs(error) > 12 for error in errors):
                raise RuntimeError('Continuous stop completed, but position error exceeds 12 counts')
        except Exception as exc:
            status['error'] = str(exc)
            record({'type': 'smooth_error', 'message': str(exc)})
        finally:
            halt()
            with lock:
                status['moving'] = False

    def jog_worker(axis, direction, step_counts, nested=False):
        try:
            before = list(checked()['encoder_counts'])
            q = before[axis]*(1 if axis == 0 else -1)
            signed_counts = validate_jog(axis, direction, q, step_counts)
            pwm = (150, 200)[axis] if direction > 0 else (120, 140)[axis]
            pwm = scaled_pwm(pwm, status['pwm_percent'])
            if cancelled.is_set():
                raise RuntimeError('Stopped')
            step.publish(Int16MultiArray(data=[axis, signed_counts, pwm]))
            record({'type': 'jog_command', 'axis': axis, 'counts': signed_counts,
                    'pwm': pwm, 'before': before})
            end = time.monotonic()+1.2
            moved = False
            while time.monotonic() < end:
                if cancelled.wait(.01):
                    raise RuntimeError('Stopped')
                s = checked()
                delta = (s['encoder_counts'][axis]-before[axis])*(1 if axis == 0 else -1)
                if abs(delta) > step_counts+12 or delta*direction < -2:
                    raise RuntimeError('Jog displacement/direction guard')
                if abs(s['encoder_counts'][1-axis]-before[1-axis]) > 2:
                    raise RuntimeError('Unexpected other-axis movement')
                moved |= abs(delta) >= min(2, step_counts)
                if moved and not any(s['applied_commands']):
                    break
            halt()
            if not moved:
                raise RuntimeError('No encoder response; motor stopped')
            end = time.monotonic()+.3
            while time.monotonic() < end:
                if cancelled.wait(.01):
                    raise RuntimeError('Stopped')
                checked()
        except Exception as exc:
            status['error'] = str(exc)
        finally:
            halt()
            record({'type': 'jog_finished', 'axis': axis, 'direction': direction,
                    'encoder_counts': latest[0]['encoder_counts'] if latest[0] else None,
                    'error': status['error']})
            if not nested:
                with lock:
                    status['moving'] = False

    def pose_worker(pose, targets):
        try:
            s = checked()
            initial = [s['encoder_counts'][0], -s['encoder_counts'][1]]
            directions = [1 if targets[i] > initial[i] else -1 for i in (0, 1)]
            done = [initial[i] <= targets[i]+4 if pose == 'open' else abs(targets[i]-initial[i]) <= 4
                    for i in (0, 1)]
            started = time.monotonic()
            record({'type': 'pose_started', 'pose': pose, 'initial': initial,
                    'targets': targets, 'pwm_percent': status['pwm_percent']})
            for _ in range(80):
                for axis in (0, 1):
                    if cancelled.is_set():
                        raise RuntimeError('Stopped')
                    if time.monotonic()-started > 90:
                        raise RuntimeError('OPEN/CLOSE deadline')
                    s = checked()
                    q = s['encoder_counts'][axis]*(1 if axis == 0 else -1)
                    if directions[axis]*(targets[axis]-q) <= 4:
                        done[axis] = True
                    if done[axis]:
                        continue
                    jog_worker(axis, directions[axis], min(5, abs(targets[axis]-q)), nested=True)
                    if status['error']:
                        raise RuntimeError(status['error'])
                if all(done):
                    break
            else:
                raise RuntimeError('OPEN/CLOSE step budget')
            s = checked()
            status['pose_result'] = {'pose': pose, 'targets': targets,
                                     'actual': [s['encoder_counts'][0], -s['encoder_counts'][1]]}
            record({'type': 'pose_finished', **status['pose_result']})
        except Exception as exc:
            status['error'] = str(exc)
            record({'type': 'pose_error', 'message': str(exc)})
        finally:
            halt()
            with lock:
                status['moving'] = False

    def worker(target):
        try:
            initial_q = checked()['encoder_counts'][0]
            if abs(target-initial_q) <= 4:
                return
            direction = 1 if target > initial_q else -1
            pwm = 150 if direction > 0 else 120
            end = time.monotonic()+12
            other = checked()['encoder_counts'][1]
            for _ in range(8):
                if cancelled.is_set():
                    raise RuntimeError('Stopped')
                s = checked()
                q = s['encoder_counts'][0]
                if direction*(target-q) <= 4:
                    return
                start_q = q
                count = min(5, abs(target-q))
                step.publish(Int16MultiArray(data=[0, -direction*count, pwm]))
                record({'type': 'command', 'axis': 0, 'target': target,
                        'counts': -direction*count, 'pwm': pwm})
                pulse_end = time.monotonic()+1.2
                moved = False
                while time.monotonic() < pulse_end:
                    if cancelled.wait(.01):
                        raise RuntimeError('Stopped')
                    s = checked()
                    delta = s['encoder_counts'][0]-start_q
                    if abs(delta) > 17 or delta*direction < -2:
                        raise RuntimeError('Step travel/direction guard')
                    if abs(s['encoder_counts'][1]-other) > 2:
                        raise RuntimeError('Unexpected second-axis movement')
                    moved |= abs(delta) >= 2
                    if moved and not any(s['applied_commands']):
                        break
                halt()
                if not moved:
                    raise RuntimeError('No encoder response; no automatic PWM increase')
                # Short settling pause; no continuous pulse or reversal chasing.
                pause_end = time.monotonic()+.25
                while time.monotonic() < pause_end:
                    if cancelled.wait(.01):
                        raise RuntimeError('Stopped')
                    checked()
                if time.monotonic() > end:
                    raise RuntimeError('Motion deadline')
            raise RuntimeError('Step budget exceeded')
        except Exception as exc:
            status['error'] = str(exc)
        finally:
            halt()
            with lock:
                status['moving'] = False
            record({'type': 'motion_finished', 'target': target, 'status': dict(status)})

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, value, content_type='application/json'):
            body = value if isinstance(value, bytes) else json.dumps(value).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path == '/':
                    self.reply(200, (ROOT / 'web/final_open_close.html').read_bytes(), 'text/html; charset=utf-8')
                    return
                if self.path == '/manual':
                    page = (ROOT / 'ros2_ws/src/rosaia_dashboard/web/index.html').read_text()
                    page = page.replace('Encoder Arm 1', 'Encoder asse 0 · ARM 2')
                    page = page.replace('Encoder Arm 2', 'Encoder asse 1 · non verificato')
                    page = page.replace('Arm 1 <b id="cur-1"', 'Asse 0 <b id="cur-1"')
                    page = page.replace('Arm 2 <b id="cur-2"', 'Asse 1 <b id="cur-2"')
                    page = page.replace('Seleziona il braccio e fai clic per una zona punta.',
                                        'Demo asse 0 (ARM 2); secondo asse fermo.')
                    page = page.replace('</head>', '<style>.legend span:not(:first-child){display:none}</style></head>')
                    extra = """<script>
                    const simpleButton=document.querySelector('#open-close');
                    const simpleLabel=document.querySelector('#open-close-status');
                    const encoderControls=document.createElement('div');
                    encoderControls.id='encoder-controls';
                    encoderControls.style.cssText='display:none!important';
                    encoderControls.innerHTML='<label for="encoder-target">Encoder target [count, 0–30]</label><input id="encoder-target" type="number" min="0" max="30" step="1" value="20" style="width:90px"><button id="send-encoder">Invia</button><button id="save-open">Salva OPEN</button><button id="save-close">Salva CLOSE</button><span id="encoder-feedback" role="status"></span>';
                    simpleLabel.after(encoderControls);
                    simpleButton.style.display='';
                    let smoothNextPose='close';
                    const jogControls=document.createElement('div');
                    jogControls.id='jog-controls';
                    jogControls.style.cssText='display:flex!important;gap:20px;flex-wrap:wrap;width:100%';
                    jogControls.innerHTML=[0,1].map(axis=>`<div class="card"><h2>Motore ${axis} · ${axis===0?'camera ARM 2':'braccio da identificare'}</h2><p id="jog-value-${axis}">Encoder —</p><label>Step [conteggi per clic, 1–10] <input id="jog-step-${axis}" type="number" min="1" max="10" step="1" value="5" style="width:80px"></label><p><button class="jog" data-axis="${axis}" data-dir="1" style="font-size:28px;min-width:80px">+</button> <button class="jog" data-axis="${axis}" data-dir="-1" style="font-size:28px;min-width:80px">−</button></p><p>Nessun limite software alla posizione totale. Un clic = un passo.</p><button class="jog-save" data-axis="${axis}" data-pose="open">Salva OPEN attuale</button> <button class="jog-save" data-axis="${axis}" data-pose="close">Salva CLOSE attuale</button><p id="jog-saved-${axis}"></p></div>`).join('');
                    encoderControls.after(jogControls);
                    const encoderInput=document.querySelector('#encoder-target');
                    const feedback=document.querySelector('#encoder-feedback');
                    feedback.style.cssText='display:block!important';
                    jogControls.after(feedback);
                    let jogPending=false;
                    async function simpleRequest(path,payload={}){
                      try{const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
                        const result=await response.json();if(!response.ok)throw new Error(result.error||'Comando rifiutato');
                        feedback.textContent=result.message||`Target inviato: ${result.target_q}`;
                      }catch(error){feedback.textContent=error.message;}
                    }
                    jogControls.querySelectorAll('.jog').forEach(button=>button.onclick=async()=>{
                      const stepInput=document.querySelector('#jog-step-'+button.dataset.axis);
                      if(jogPending||!stepInput.reportValidity())return;jogPending=true;
                      jogControls.querySelectorAll('button').forEach(b=>b.disabled=true);
                      try{await simpleRequest('/simple/jog',{axis:Number(button.dataset.axis),direction:Number(button.dataset.dir),step_counts:Number(stepInput.value)});}
                      finally{jogPending=false;}
                    });
                    jogControls.querySelectorAll('.jog-save').forEach(button=>button.onclick=()=>
                      simpleRequest('/simple/jog-nominal',{axis:Number(button.dataset.axis),pose:button.dataset.pose}));
                    document.querySelector('#send-encoder').onclick=()=>{
                      if(!encoderInput.reportValidity())return;
                      simpleRequest('/simple/position',{target:encoderInput.value});};
                    for(const pose of ['open','close'])document.querySelector('#save-'+pose).onclick=()=>{
                      if(!encoderInput.reportValidity())return;
                      simpleRequest('/simple/nominal',{pose,target:encoderInput.value});};
                    document.querySelector('#front-view').onclick=null;
                    simpleButton.onclick=async()=>{simpleButton.disabled=true;
                      await simpleRequest('/simple/smooth',{pose:smoothNextPose});};
                    document.querySelector('#stop-all').onclick=async()=>{
                      await fetch('/simple/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});};
                    async function simplePoll(){try{const s=await(await fetch('/simple/status')).json();
                      simpleButton.disabled=s.moving||!s.ready||!s.smooth_available;
                      encoderControls.querySelectorAll('button').forEach(b=>b.disabled=s.moving||!s.ready);
                      jogControls.querySelectorAll('button').forEach(b=>b.disabled=jogPending||s.moving||!s.ready);
                      for(const axis of [0,1]){
                        document.querySelector('#jog-value-'+axis).textContent=`Posizione ${s.jog_positions?.[axis]??'—'} · encoder raw ${s.encoders_raw?.[axis]??'—'}`;
                        const saved=s.jog_nominals?.[axis]||{};
                        document.querySelector('#jog-saved-'+axis).textContent=`OPEN ${saved.open?.q??'—'} / CLOSE ${saved.close?.q??'—'}`;
                      }
                      smoothNextPose=s.smooth_next_pose||'close';
                      simpleButton.textContent=!s.smooth_available?'Continuo · da verificare':s.moving?'MOVIMENTO…':smoothNextPose==='open'?'OPEN · entrambi':'CLOSE · entrambi';
                      simpleLabel.textContent=(s.error?s.error+' · ':'')+'+ contrae, − rilascia · step modificabile · nessun limite software di posizione · STOP/corrente/timeout attivi · dati registrati';
                    }catch(e){simpleButton.disabled=true;jogControls.querySelectorAll('button').forEach(b=>b.disabled=true);encoderControls.querySelectorAll('button').forEach(b=>b.disabled=true);simpleLabel.textContent='Controllo offline';}}
                    setInterval(simplePoll,250);simplePoll();
                    </script>"""
                    self.reply(200, page.replace('</body>', extra+'</body>').encode(), 'text/html; charset=utf-8')
                elif self.path == '/simple/status':
                    try:
                        q = checked()['encoder_counts'][0]
                        ready = True
                    except RuntimeError:
                        q = 0
                        ready = False
                    try:
                        closes = smooth_targets('close')
                        current = [latest[0]['encoder_counts'][0], -latest[0]['encoder_counts'][1]] if ready else [0,0]
                        next_pose = 'open' if any(current[i] > closes[i]/2 for i in (0,1)) else 'close'
                        smooth_available = CONTINUOUS_ENABLED
                    except ValueError:
                        next_pose, smooth_available = 'close', False
                    self.reply(200, {**status, 'ready': ready,
                                     'smooth_available': smooth_available, 'smooth_next_pose': next_pose,
                                     'encoders_raw': latest[0]['encoder_counts'] if ready else None,
                                     'jog_positions': [latest[0]['encoder_counts'][0], -latest[0]['encoder_counts'][1]] if ready else None,
                                     'jog_nominals': jog_nominals,
                                     'encoder_q': q if ready else None,
                                     'next_action': 'CLOSE' if q <= status['open_q']+4 else 'OPEN'})
                elif self.path == '/api/state' or self.path.startswith('/api/camera.jpg'):
                    with urlopen('http://127.0.0.1:8765'+self.path, timeout=2) as response:
                        self.reply(200, response.read(), response.headers.get('Content-Type', 'application/octet-stream'))
                else:
                    self.reply(404, {'error': 'not found'})
            except Exception as exc:
                self.reply(503, {'error': str(exc)})

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin not in (None, 'http://127.0.0.1:8767', 'http://localhost:8767'):
                self.reply(403, {'error': 'origin rejected'})
                return
            if self.path == '/simple/stop':
                cancelled.set()
                halt()
                self.reply(200, {'stopped': True})
                return
            if self.path not in ('/simple/toggle', '/simple/position', '/simple/nominal',
                                 '/simple/jog', '/simple/jog-nominal', '/simple/smooth',
                                 '/simple/pose', '/simple/pwm'):
                self.reply(404, {'error': 'not found'})
                return
            with lock:
                try:
                    if status['moving']:
                        raise RuntimeError('Motion already active')
                    s = checked()
                    if any(s['applied_commands']):
                        raise RuntimeError('Motors already active')
                    if self.path == '/simple/toggle':
                        target = next_target(s['encoder_counts'][0], status['open_q'], status['close_q'])
                    else:
                        length = int(self.headers.get('Content-Length', '0'))
                        if not 0 < length <= 2048:
                            raise ValueError('Invalid request length')
                        payload = json.loads(self.rfile.read(length))
                        if not isinstance(payload, dict):
                            raise ValueError('JSON object required')
                        if self.path == '/simple/pwm':
                            percent = payload.get('percent')
                            scaled_pwm(200, percent)
                            record({'type': 'pwm_intensity_changed', 'previous': status['pwm_percent'], 'percent': percent})
                            status['pwm_percent'] = percent
                            self.reply(200, {'message': f'Intensita PWM: {percent}%. Motori fermi.', 'percent': percent})
                            return
                        if self.path == '/simple/pose':
                            pose = payload.get('pose')
                            targets = smooth_targets(pose)
                            cancelled.clear()
                            status.update(moving=True, error=None, pose_result=None)
                            threading.Thread(target=pose_worker, args=(pose, targets), daemon=True).start()
                            self.reply(202, {'message': f'{pose.upper()} avviato', 'targets': targets})
                            return
                        if self.path == '/simple/smooth':
                            if not CONTINUOUS_ENABLED:
                                raise RuntimeError('Continuous CLOSE is not validated: host-current protection intervened; use bounded manual jog')
                            pose = payload.get('pose')
                            targets = smooth_targets(pose)
                            cancelled.clear()
                            status.update(moving=True, error=None, smooth_result=None)
                            threading.Thread(target=smooth_worker, args=(pose, targets), daemon=True).start()
                            self.reply(202, {'message': f'{pose.upper()}: movimento continuo di entrambi',
                                             'targets': targets})
                            return
                        if self.path in ('/simple/jog', '/simple/jog-nominal'):
                            axis = payload.get('axis')
                            if type(axis) is not int or axis not in (0, 1):
                                raise ValueError('Select motor axis 0 or 1')
                            q = s['encoder_counts'][axis]*(1 if axis == 0 else -1)
                            if self.path == '/simple/jog-nominal':
                                pose = payload.get('pose')
                                if pose not in ('open', 'close'):
                                    raise ValueError('Select OPEN or CLOSE')
                                saved = dict(jog_nominals.get(str(axis), {}))
                                saved[pose] = {'q': q, 'raw': s['encoder_counts'][axis],
                                               'saved_at_ns': time.time_ns()}
                                if 'open' in saved and 'close' in saved and saved['open']['q'] >= saved['close']['q']:
                                    raise ValueError('OPEN must be smaller than CLOSE')
                                updated = {**jog_nominals, str(axis): saved}
                                save_nominal_file(jog_nominal_path, updated)
                                jog_nominals.update(updated)
                                record({'type': 'jog_nominal_saved', 'axis': axis, 'pose': pose, 'saved': saved})
                                self.reply(200, {'message': f'Motore {axis}: {pose.upper()} salvato a {q}. Nessun movimento.'})
                                return
                            direction = payload.get('direction')
                            step_counts = payload.get('step_counts', 5)
                            signed_counts = validate_jog(axis, direction, q, step_counts)
                            cancelled.clear()
                            status.update(moving=True, error=None)
                            threading.Thread(target=jog_worker, args=(axis, direction, step_counts), daemon=True).start()
                            self.reply(202, {'message': f'Motore {axis}: passo {"+" if direction > 0 else "−"} avviato',
                                             'axis': axis, 'signed_motor_counts': signed_counts})
                            return
                        target = validate_target(payload.get('target'))
                    if self.path == '/simple/nominal':
                        pose = payload.get('pose')
                        if pose not in ('open', 'close'):
                            raise ValueError('Select OPEN or CLOSE')
                        if abs(target-s['encoder_counts'][0]) > 4:
                            raise ValueError('Invia prima il target e verifica la posizione prima di salvarla')
                        open_q = target if pose == 'open' else status['open_q']
                        close_q = target if pose == 'close' else status['close_q']
                        open_q, close_q = validate_nominals(open_q, close_q)
                        saved = {'axis': 0, 'camera_arm': 'arm_2',
                                 'open_q': open_q, 'close_q': close_q,
                                 'captured_encoder_q': s['encoder_counts'][0],
                                 'saved_at_ns': time.time_ns(),
                                 'note': 'Entered nominal targets; recheck after HOME/tendon changes'}
                        save_nominal_file(nominal_path, saved)
                        record({'type': 'nominals_saved', 'previous_open_q': status['open_q'],
                                'previous_close_q': status['close_q'], 'saved': saved})
                        status.update(open_q=open_q, close_q=close_q)
                        self.reply(200, {'message': f'{pose.upper()} salvato: {target}. Nessun movimento.', **saved})
                        return
                    cancelled.clear()
                    status.update(moving=True, error=None)
                    threading.Thread(target=worker, args=(target,), daemon=True).start()
                    self.reply(202, {'target_q': target})
                except (RuntimeError, ValueError) as exc:
                    self.reply(409, {'error': str(exc)})

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 8767), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    def end_signal(*_args):
        cancelled.set()
        shutdown.set()
    signal.signal(signal.SIGINT, end_signal)
    signal.signal(signal.SIGTERM, end_signal)
    print('Simple OPEN/CLOSE: http://127.0.0.1:8767', flush=True)
    print('Recorded signals:', folder, flush=True)
    try:
        while not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        cancelled.set()
        halt()
        server.shutdown()
        # Workers finish after cancellation before log/subscription teardown.
        deadline = time.monotonic()+2
        while status['moving'] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
