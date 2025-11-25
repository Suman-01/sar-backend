// src/main.rs
use std::process::Command;  // import to run external processes in tmux
use std::path::PathBuf;     // filesystem safe path builder
use std::env;               // read the env variables

//Start a tmux session
fn start_tmux(args: &[&str]) -> Result<(), String> { 

    // stat variable to hold command status
    let stat = Command::new("tmux")   // run tmux with the given args
        .args(args)     // the args
        .status()       // execute and wait
        .map_err(|e| format!("failed to run tmux {:?}: {}", args, e))?;     // error handling

    // check if success
    if !stat.success() {
        return Err(format!("tmux {:?} failed with {}", args, stat)); //err if !success
    }

    Ok(())  
}

// create a new tmux session and run a cmd
fn new_session(session: &str, window_name: &str, first_cmd: &str) -> Result<(), String> {
    // run <tmux new-session -d -s <session> -n <window_name> "<first_cmd>"
    let args = ["new-session", "-d", "-s", session, "-n", window_name, first_cmd];
    start_tmux(&args)
}

// create a new tmux window in the given session
fn new_window(session: &str, window_name: &str) -> Result<(), String> {

    // run <tmux new-window -t <session> -n <window_name>>
    start_tmux(&["new-window", "-t", session, "-n", window_name])
}

// send cmd to the target tmux pane/window
fn send_keys(session: &str, target: &str, cmd: &str) -> Result<(), String> {   
    
    //send the command <tmux send-keys -t <session>:<target> "<cmd>" C-m> to the target pane
    start_tmux(&[
        "send-keys", 
        "-t", 
        &format!("{}:{}", session, target), 
        cmd, 
        "C-m"
    ])
}

// kill existing tmux session
fn kill_session(session: &str) -> Result<(), String> {
    start_tmux(&["kill-session", "-t", session])
}

fn main() -> Result<(), String> {

    // CONFIG
    let session = "px4_multi";      // session name
    let workspace = PathBuf::from("/home/admin1/prj");      // PX4-Autopilot path
    let px4_path = workspace.join("PX4-Autopilot").join("build/px4_sitl_default/bin/px4");      // px4 binary path

// *-> check why pathbuf is safe!

    // px4 binary check
    if !px4_path.exists() {
        return Err(format!(
            "px4 binary not found at {}.",
            px4_path.display()
        ));
    }

// *-> supposed to be dynamic. (input no. of drones and it automatically creates tmux sessions for them)
   
    // cmd to run in tmux terminal 1 (server)
    let cmd1 = format!(
        "cd {} && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 {} -i 1",
        workspace.join("PX4-Autopilot").display(),
        px4_path.display()
    );

    // cmd to run in tmux terminal 2 (client)
    let cmd2 = format!(
        "cd {} && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,1\" PX4_SIM_MODEL=gz_x500 {} -i 2",
        workspace.join("PX4-Autopilot").display(),
        px4_path.display()
    );

    // Safety: Avoid Collision. if a session with same name already exists, kill it
    let _ = kill_session(session); // ignore err : session doesn't exist
    
    println!("Creating tmux session '{}' ...", session);    
// *-> loads the bashrc and all env variables thus local venv doesn't work. (fix this)
    let first_cmd = format!("bash -lc '{}; exec bash'", cmd1.replace('\'', "\\'"));     // var to hold first cmd with bash login shell
    new_session(session, "px4-1", &first_cmd)?;     // Start session - create window 1 - name px4-1 - run cmd1
    new_window(session, "px4-2")?;      // create window 2 - name px4-2
    send_keys(session, "px4-2", &format!("bash -lc '{}; exec bash'", cmd2.replace('\'', "\\'")))?;      // send cmd2 to window 2

   

    println!("Launched px4 multi-vehicle tmux session '{}'.", session);
    println!("... Window 0 - px4-1 ...");
    println!("... Window 1 - px4-2 ...");
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
