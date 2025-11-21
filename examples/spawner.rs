// src/main.rs
use std::process::Command;
use std::path::PathBuf;
use std::env;

fn run_tmux(args: &[&str]) -> Result<(), String> {
    let status = Command::new("tmux")
        .args(args)
        .status()
        .map_err(|e| format!("failed to run tmux {:?}: {}", args, e))?;
    if !status.success() {
        return Err(format!("tmux {:?} failed with {}", args, status));
    }
    Ok(())
}

fn tmux_send_keys(session: &str, target: &str, cmd: &str) -> Result<(), String> {
    // use send-keys ... C-m to press enter
    run_tmux(&["send-keys", "-t", &format!("{}:{}", session, target), cmd, "C-m"])
}

fn tmux_new_session(session: &str, window_name: &str, first_cmd: &str) -> Result<(), String> {
    // new-session -d -s <session> -n <window_name> "<first_cmd>"
    let args = ["new-session", "-d", "-s", session, "-n", window_name, first_cmd];
    run_tmux(&args)
}

fn tmux_new_window(session: &str, window_name: &str) -> Result<(), String> {
    run_tmux(&["new-window", "-t", session, "-n", window_name])
}

fn tmux_kill_session(session: &str) -> Result<(), String> {
    run_tmux(&["kill-session", "-t", session])
}

fn main() -> Result<(), String> {
    // CONFIG: edit these if your layout differs
    let session = "px4_multi"; // tmux session name
    let workspace = PathBuf::from("/home/admin1/prj"); // path that contains PX4-Autopilot
    let px4_path = workspace.join("PX4-Autopilot").join("build/px4_sitl_default/bin/px4");

    if !px4_path.exists() {
        return Err(format!(
            "px4 binary not found at {}. Build with `make px4_sitl` first.",
            px4_path.display()
        ));
    }

    // Commands to run in terminals (exactly per your example)
    // Terminal 1 (server)
    let cmd1 = format!(
        "cd {} && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 {} -i 1",
        workspace.join("PX4-Autopilot").display(),
        px4_path.display()
    );

    // Terminal 2 (client connecting to server)
    let cmd2 = format!(
        "cd {} && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,1\" PX4_SIM_MODEL=gz_x500 {} -i 2",
        workspace.join("PX4-Autopilot").display(),
        px4_path.display()
    );

    // Safety: if a session already exists, kill it (ask user)
    // We'll attempt to kill any existing session with same name to avoid collisions.
    let _ = tmux_kill_session(session); // ignore error if session didn't exist

    println!("Creating tmux session '{}' ...", session);
    // Create session with first window named "px4-1" and start cmd1 directly in it:
    // using new-session -d -s session -n window "bash -lc '<cmd>'"
    let first_cmd = format!("bash -lc '{}; exec bash'", cmd1.replace('\'', "\\'"));
    tmux_new_session(session, "px4-1", &first_cmd)?;

    // Create a second window and run second command
    tmux_new_window(session, "px4-2")?;
    tmux_send_keys(session, "px4-2", &format!("bash -lc '{}; exec bash'", cmd2.replace('\'', "\\'")))?;

    // Optional: you can create more windows (e.g., for logs or other utilities) similarly.

    println!("Launched px4 multi-vehicle tmux session '{}'.", session);
    println!(" - Window 0 (px4-1) runs server (instance 1)");
    println!(" - Window 1 (px4-2) runs client (instance 2)");
    println!();
    println!("Attach to session to view output: tmux attach -t {}", session);
    println!("Detach with Ctrl-b d. Kill session with: tmux kill-session -t {}", session);

    // If user wants to auto-attach we can do that — but we leave it detached so you can inspect steps.
    // Optionally attach now:
    if env::var("AUTO_ATTACH").is_ok() {
        println!("AUTO_ATTACH detected, attaching to tmux session...");
        let _ = Command::new("tmux").args(&["attach-session", "-t", session]).status();
    }

    Ok(())
}
