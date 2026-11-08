"""Python 3 Pi-host helper, sent over SSH by the laptop dashboard.

Only Git working files are changed. This never starts or stops onboard nodes.
"""
import json
import os
import re
import subprocess
import sys


def command(*args):
    environment = dict(os.environ, GIT_TERMINAL_PROMPT='0',
                       GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=5')
    return subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True, timeout=30, env=environment)


def git(*args):
    result = command('git', *args)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or 'Git command failed')
    return result.stdout.strip()


def state():
    branch = command('git', 'symbolic-ref', '--quiet', '--short', 'HEAD')
    return {'branch': branch.stdout.strip() or '(detached)',
            'revision': git('rev-parse', '--short', 'HEAD'),
            'dirty': bool(git('status', '--porcelain'))}


def status():
    result = state()
    refs = git('ls-remote', '--heads', 'origin')
    result['branches'] = sorted(line.split('\t', 1)[1][len('refs/heads/'):]
                                for line in refs.splitlines() if '\trefs/heads/' in line)
    return result


def check_worktree(require_clean=True):
    changes = git('status', '--porcelain')
    if require_clean and changes:
        raise RuntimeError('Local changes: save or stash them in the terminal, then retry.\n' + changes)
    routine = command('docker', 'exec', 'pidrone_pkg', 'pgrep', '-f', 'PALS/routines/[A-Za-z0-9_-]+[.]py')
    if routine.returncode != 1:
        raise RuntimeError('Stop the running routine first.' if routine.returncode == 0
                           else 'Could not check onboard routines; connect the drone first.')
    mode = command('docker', 'exec', 'pidrone_pkg', 'bash', '-ic',
                   'timeout 4 rostopic echo -n1 /pidrone/mode')
    if mode.returncode or not re.search(r'^mode: [\"\']?DISARMED[\"\']?\s*$', mode.stdout, re.M):
        raise RuntimeError('The drone must report DISARMED. Connect it while disarmed, then retry.')


def target(branch):
    git('check-ref-format', '--branch', branch)
    remote = 'refs/remotes/origin/' + branch
    revision = git('rev-parse', '--verify', remote)
    local = 'refs/heads/' + branch
    if command('git', 'show-ref', '--verify', '--quiet', local).returncode == 0:
        if command('git', 'merge-base', '--is-ancestor', local, remote).returncode:
            raise RuntimeError('Local branch has commits not on origin; reconcile it in the terminal first.')
    return revision


def change(action, branch=None, revision=None):
    import fcntl  # Pi host only; the laptop server does not import this helper.

    if action not in ('prepare', 'switch', 'prepare-clear', 'clear'):
        raise ValueError('Unknown branch action')
    clearing = action in ('prepare-clear', 'clear')
    if not clearing:
        git('check-ref-format', '--branch', branch)
    with open('/tmp/pals-start.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another startup or maintenance action is running; try again when it finishes.')
        check_worktree(require_clean=not clearing)
        if action == 'prepare-clear':
            return state()
        if action == 'clear':
            root = git('rev-parse', '--show-toplevel')
            git('reset', '--hard', 'HEAD')
            git('-C', root, 'clean', '-fd')
            result = state()
            if result['dirty']:
                raise RuntimeError('Some local changes could not be cleared. Check the terminal, then Refresh.')
            return result
        if action == 'prepare':
            git('fetch', '--prune', 'origin')
            return {'target': target(branch)}
        if target(branch) != revision:
            raise RuntimeError('The remote branch changed during preparation; refresh and retry.')
        if command('git', 'show-ref', '--verify', '--quiet', 'refs/heads/' + branch).returncode == 0:
            git('checkout', branch)
            git('merge', '--ff-only', '--no-edit', 'refs/remotes/origin/' + branch)
            git('branch', '--set-upstream-to=origin/' + branch, branch)
        else:
            git('checkout', '--track', '-b', branch, 'refs/remotes/origin/' + branch)
        return state()


if __name__ == '__main__':
    try:
        action = sys.argv[1]
        if action == 'status':
            result = status()
        elif action in ('prepare', 'switch'):
            result = change(action, sys.argv[2], sys.argv[3] if action == 'switch' else None)
        elif action in ('prepare-clear', 'clear'):
            result = change(action)
        else:
            raise ValueError('Unknown branch action')
        print(json.dumps(result))
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(json.dumps({'error': str(error)}))
        sys.exit(1)
