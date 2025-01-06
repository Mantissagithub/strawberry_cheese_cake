import os
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
from multiprocessing import Process


class ROS2GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 Multi-Process GUI")

        # Track running processes
        self.processes = {}

        # Create a frame for script execution
        exec_frame = ttk.LabelFrame(root, text="Run Scripts", padding=(10, 10))
        exec_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        ttk.Button(exec_frame, text="Run Input1", command=lambda: self.run_script("input1")).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(exec_frame, text="Run Input2", command=lambda: self.run_script("input2")).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(exec_frame, text="Run Controller", command=lambda: self.run_script("controller")).grid(row=0, column=2, padx=5, pady=5)

        ttk.Button(exec_frame, text="Stop All Scripts", command=self.stop_all_scripts).grid(row=1, column=0, columnspan=3, pady=5)

        # Create a button for colcon build
        ttk.Button(root, text="Colcon Build", command=self.colcon_build).grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        # Create a button for sourcing workspace
        ttk.Button(root, text="Source Workspace", command=self.source_workspace).grid(row=2, column=0, padx=10, pady=5, sticky="ew")

    def run_script(self, script_name):
        """Run the specified script as a ROS 2 executable."""
        if script_name in self.processes and self.processes[script_name].is_alive():
            messagebox.showinfo("Info", f"{script_name} is already running.")
            return

        command = f"ros2 run autonomous_stack {script_name}"

        # Start the script in a separate process
        process = Process(target=self.execute_command, args=(command,))
        process.start()
        self.processes[script_name] = process
        messagebox.showinfo("Running", f"{script_name} started successfully.")

    def stop_all_scripts(self):
        """Stop all running scripts."""
        stopped_scripts = []
        for script_name, process in list(self.processes.items()):
            if process.is_alive():
                process.terminate()
                process.join()
                stopped_scripts.append(script_name)
                del self.processes[script_name]

        if stopped_scripts:
            messagebox.showinfo("Stopped", f"Stopped: {', '.join(stopped_scripts)}")
        else:
            messagebox.showinfo("Info", "No scripts are currently running.")

    def colcon_build(self):
        """Trigger a colcon build."""
        if any(process.is_alive() for process in self.processes.values()):
            messagebox.showerror("Error", "Stop all running scripts before building.")
            return

        try:
            subprocess.run("colcon build", shell=True, check=True)
            messagebox.showinfo("Success", "Colcon build completed successfully.")
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Error", f"Colcon build failed.\n{e}")

    def source_workspace(self):
        """Source the ROS 2 workspace directly in the current terminal."""
        try:
            command = "source ~/irc_autonomous_ws/install/setup.bash"
            subprocess.run(command, shell=True, check=True, executable="/bin/bash")
            messagebox.showinfo("Success", "Workspace sourced successfully.")
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Error", f"Failed to source workspace.\n{e}")

    @staticmethod
    def execute_command(command):
        """Execute a shell command."""
        subprocess.run(command, shell=True)


if __name__ == "__main__":
    root = tk.Tk()
    gui = ROS2GUI(root)
    root.mainloop()
