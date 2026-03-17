use std::{
    os::unix::process::CommandExt,
    process::{Child, Command},
    time::Duration,
};

/// Spawn child in its own process group with orphan prevention.
///
/// Sets `PR_SET_PDEATHSIG(SIGKILL)` so the child is killed if the parent dies,
/// and creates a new process group so we can signal the entire group on cleanup.
pub fn set_new_process_group(command: &mut Command) -> &mut Command {
    unsafe {
        command.pre_exec(|| {
            // Create new process group
            libc::setpgid(0, 0);
            // Kill this process if parent dies (Linux-specific)
            #[cfg(target_os = "linux")]
            libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL);
            Ok(())
        })
    }
}

/// Gracefully kill the entire process group: SIGTERM, wait, then SIGKILL.
pub fn graceful_kill_process_group(child: &mut Child) {
    let pid = child.id() as libc::pid_t;
    let pgid = -pid; // negative pid = entire process group

    // Send SIGTERM to the process group
    unsafe {
        libc::kill(pgid, libc::SIGTERM);
    }

    // Wait up to 5 seconds for graceful shutdown
    for _ in 0..50 {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => std::thread::sleep(Duration::from_millis(100)),
            Err(_) => return,
        }
    }

    // Force kill the entire process group
    unsafe {
        libc::kill(pgid, libc::SIGKILL);
    }
    let _ = child.wait();
}
