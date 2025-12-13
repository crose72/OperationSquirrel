# squirreldefender Docker Container

For deploying the code to a production environment you may prefer to have the program start as soon as you power the jetson.  There is a container built to enable that.  It is executed by a service that runs on startup.

## Release container:

If you don't need to change the code and just want to run the precompiled program then you can just do the setup and run `./run.sh squirreldefender orin` or `./run.sh squirreldefender b01` to run the program for your Jetson.  The purpose of this container is to be deployed onto your drone or other autonomous vehicle since the only thing that it does is run the pre-compiled binary that you created using the dev container from the previous step.

If you want to have the program run as soon as you power on the jetson (for example when your jetson is mounted on your drone and you plug in the battery to your jetson) then you need to do the setup, and then enable the `squirreldefender.service` by doing

```bash
sudo systemctl enable squirreldefender.service
```

Now your squirreldefender program should start running every time your jetson is powered on (until you disable the service), and if your jetson is connected to the drone or the simulation on your laptop via FTDI they you'll see some action.  You can also stop, restart, or check the status of the program by using those commands if you need to troubleshoot.  These steps also mean that as soon as you power on your jetson this program will start running unless you disable it or stop it manually.  Log files from this release container are stored in `~/logs` by default.

Some additional useful commands

```bash
# Save the file and reload the daemon
sudo systemctl daemon-reload

# Enable the service on startup
sudo systemctl enable squirreldefender.service

# Disable the service on startup
sudo systemctl disable squirreldefender.service

# Start the service
sudo systemctl start squirreldefender.service

# Stop the service
sudo systemctl stop squirreldefender.service

# Restart the service
sudo systemctl restart squirreldefender.service

# Check status of the service
sudo systemctl status squirreldefender.service
```

### Quickly deploying code changes to the drone:

Let's say you're at the park with your jetson and your drone, and you want to try out different PID gains on the follow algorithm in real life.  Or maybe you want to change the follow distance, or test out some other code changes on the real drone.  How can you make a change to the code, recompile, and update the release container, and then just power the jetson and fly?

***Remember to disconnect the uart wires or disconnect the battery from the drone so that the props don't start spinning when you're not ready (if squirreldefender.service is active and started/restarts when you power on the jetson then it will send the command to arm the drone and takeoff so you don't want it near you when that happens)

1. Stop squirreldefender service
    - `sudo systemctl stop squirreldefender.service`
2. Modify the code using the dev container (follow instructions above)
    - Execute `./run.sh dev orin` (or whatever jetson you're using)
    - Make changes to the code
    - Recompile the code
3. Build the squirreldefender container
    - Execute `./build.sh squirreldefender orin` (or whatever jetson you're using)
    - (Choose N if you don't need to push the container to docker hub - since you're at the park I don't think you do)
4. Enable the squirreldefender service (if not already enabled)
    - `sudo systemctl enable squirreldefender.service`


### Clock skew error when Jetson no longer has wifi connection

When the Jetson has no internet connection the system time defaults to 1969.  Since you're probably working in the present (2025 and beyond), any files you've touched will likey have a timestamp in the present.  This timestamp is after 1969 by a lot.  So if you go to compile the squirreldefender program the compiler will notice that the files have a timestamp millions of seconds in the future, resulting in a clock skew when recompiling.  To solve this issue, create a systemd service to automatically move the system time to a few seconds after the latest timestamp in the operationsquirrel repo.

The `setup.sh` should take care of this, but here is the manual setup for your benefit.

Create a systemd service

```bash
sudo nano /etc/systemd/system/clock-skew-fix.service
```

```bash
# Add this to the file

    [Unit]
    Description=Ensure system clock is ahead of all file timestamps
    After=multi-user.target
    [Service]
    Type=oneshot
    ExecStart=/bin/bash -c '
      # Replace with your actual project/code path
      latest=$(find /home/user/project -type f -exec stat -c %%Y {} + 2>/dev/null | sort -n | tail -1)
      now=$(date +%%s)
      if [ -n "$latest" ] && [ "$now" -lt "$latest" ]; then
        echo "⏩ Clock behind file timestamps, setting time forward..."
        date -s "@$((latest + 10))"
      fi
    '
    [Install]
    WantedBy=multi-user.target
```

Next, enable and start the systemd service you just created

```
sudo systemctl enable clock-skew-fix.service
sudo systemctl start clock-skew-fix.service
```