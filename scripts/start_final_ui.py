#!/usr/bin/env python3
"""Start/attach to the final UI; require explicit HOME confirmation on a fresh stack."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import webbrowser

from rosaia_workflow import ROOT, api
from urllib.request import Request, urlopen


URL = 'http://127.0.0.1:8767'


def status():
    with urlopen(URL+'/simple/status', timeout=2) as response:
        return json.load(response)


def owned_backends():
    target = str(ROOT / 'scripts/simple_open_close.py').encode()
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            if path.stat().st_uid == os.getuid() and target in (path/'cmdline').read_bytes().split(b'\0'):
                yield int(path.name)
        except (OSError, ProcessLookupError):
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--no-open', action='store_true')
    options = parser.parse_args()
    if options.stop:
        try:
            request = Request(URL+'/simple/stop', data=b'{}', headers={'Content-Type': 'application/json'})
            urlopen(request, timeout=2).close()
        except OSError:
            pass
        for pid in owned_backends():
            os.kill(pid, signal.SIGINT)
        print('STOP inviato; interfaccia finale arrestata. Registrazioni conservate.')
        return
    try:
        running = status()
    except OSError:
        running = None
    if running is not None:
        if not running.get('armed'):
            raise RuntimeError('Riferimento non valido: esegui 03_demo_finale.sh stop, poi start e conferma HOME')
        print('Interfaccia gia attiva:', URL)
        if not options.no_open:
            webbrowser.open(URL)
        return
    state = api('/api/state')
    if not state.get('home_calibrated'):
        answer = input('Conferma bracci nella stessa HOME meccanica della calibrazione, tendini corretti e spazio libero. Scrivi HOME per azzerare: ')
        if answer.strip().upper() != 'HOME':
            raise RuntimeError('Avvio annullato; nessun azzeramento')
        response = api('/api/home', {})
        if not response.get('success'):
            raise RuntimeError(str(response))
    log_dir = ROOT / 'data/logs' / datetime.now().strftime('final_ui_%Y%m%d_%H%M%S')
    log_dir.mkdir(parents=True)
    with (log_dir/'server.log').open('w') as output:
        process = subprocess.Popen(['bash', str(ROOT/'scripts/rosaia.sh'), 'simple-demo'],
                                   cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    deadline = time.monotonic()+15
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Avvio fallito: {log_dir}/server.log')
            try:
                running = status()
                if running.get('ready'):
                    break
            except OSError:
                pass
            time.sleep(.2)
        else:
            raise RuntimeError(f'Camera/telemetria non pronte: {log_dir}/server.log')
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        raise
    print('Demo pronta:', URL)
    print('Nessun movimento avviato. Per arrestare: bash 03_demo_finale.sh stop')
    if not options.no_open:
        webbrowser.open(URL)


if __name__ == '__main__':
    main()
