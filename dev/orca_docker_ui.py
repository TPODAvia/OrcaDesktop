#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import os

# Directory where your Makefile lives
WORKDIR = "/home/rover2/OrcaDesktops"

COMMANDS = {
    "Install images": ["make", "install"],
    "Run stack": ["make", "run"],
    "Stop stack": ["make", "stop"],
    "Remove containers (down)": ["make", "remove"],
    "Uninstall (down + volumes)": ["make", "uninstall"],
}


class DockerControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Orca Docker Control")
        self.geometry("850x480")
        self.minsize(750, 420)

        # Slightly nicer default font
        default_font = ("Segoe UI", 10)
        self.option_add("*Font", default_font)

        self.style = ttk.Style(self)
        self._configure_style()

        self._create_widgets()

    # ---------- UI / Style ----------

    def _configure_style(self):
        # Use a modern ttk theme if available
        try:
            if "clam" in self.style.theme_names():
                self.style.theme_use("clam")
        except tk.TclError:
            pass

        BG = "#20252b"
        PANEL_BG = "#252b33"
        TEXT = "#f0f0f0"
        MUTED = "#9ca3af"

        self.configure(bg=BG)

        self.style.configure("MainFrame.TFrame", background=BG)
        self.style.configure("Sidebar.TFrame", background=PANEL_BG)
        self.style.configure("Header.TFrame", background=BG)
        self.style.configure("Content.TFrame", background=PANEL_BG)

        self.style.configure(
            "Header.TLabel",
            background=BG,
            foreground=TEXT,
            font=("Segoe UI", 14, "bold"),
        )
        self.style.configure(
            "SubHeader.TLabel",
            background=BG,
            foreground=MUTED,
            font=("Segoe UI", 9),
        )
        self.style.configure(
            "Status.TLabel",
            background=PANEL_BG,
            foreground=MUTED,
            font=("Segoe UI", 9),
        )

        # Primary button style
        self.style.configure(
            "Primary.TButton",
            padding=(10, 6),
            font=("Segoe UI", 10, "bold"),
        )

        # Danger button for uninstall
        self.style.configure(
            "Danger.TButton",
            padding=(10, 6),
            font=("Segoe UI", 10, "bold"),
            foreground="#f97373",
        )

    def _create_widgets(self):
        # Main layout container
        main = ttk.Frame(self, style="MainFrame.TFrame", padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        main.columnconfigure(0, weight=0)  # sidebar
        main.columnconfigure(1, weight=1)  # content
        main.rowconfigure(1, weight=1)

        # ----- Header -----
        header = ttk.Frame(main, style="Header.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        title = ttk.Label(
            header,
            text="Orca Docker Control",
            style="Header.TLabel",
        )
        subtitle = ttk.Label(
            header,
            text="One-click control for your docker-compose stack",
            style="SubHeader.TLabel",
        )
        title.pack(anchor="w")
        subtitle.pack(anchor="w")

        # ----- Sidebar with buttons -----
        sidebar = ttk.Frame(main, style="Sidebar.TFrame", padding=(8, 10))
        sidebar.grid(row=1, column=0, sticky="nsw")
        sidebar.rowconfigure(len(COMMANDS) + 1, weight=1)  # spacer

        # Buttons
        for i, (label, cmd) in enumerate(COMMANDS.items()):
            # Only Uninstall is "danger"
            style_name = "Danger.TButton" if "Uninstall" in label else "Primary.TButton"
            btn = ttk.Button(
                sidebar,
                text=label,
                style=style_name,
                command=lambda c=cmd, l=label: self.run_command(l, c),
                width=24,
            )
            btn.grid(row=i, column=0, sticky="ew", pady=4)

        # ----- Content area -----
        content = ttk.Frame(main, style="Content.TFrame", padding=10)
        content.grid(row=1, column=1, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(1, weight=1)

        # Status bar
        self.status_var = tk.StringVar(value="Status: idle")
        status_label = ttk.Label(
            content,
            textvariable=self.status_var,
            style="Status.TLabel",
        )
        status_label.grid(row=0, column=0, sticky="ew", pady=(0, 6))

        # Log area
        log_frame = ttk.Frame(content, style="Content.TFrame")
        log_frame.grid(row=1, column=0, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log = scrolledtext.ScrolledText(
            log_frame,
            wrap=tk.WORD,
            state="disabled",
            background="#111827",
            foreground="#e5e7eb",
            insertbackground="#e5e7eb",
            borderwidth=0,
            relief=tk.FLAT,
        )
        self.log.grid(row=0, column=0, sticky="nsew")

        # Log tags for some color accents
        self.log.tag_config("cmd", foreground="#93c5fd")
        self.log.tag_config("ok", foreground="#6ee7b7")
        self.log.tag_config("err", foreground="#fca5a5")

    # ---------- Logging & command execution ----------

    def append_log(self, text: str, tag: str | None = None):
        self.log.configure(state="normal")
        if tag:
            self.log.insert(tk.END, text, tag)
        else:
            self.log.insert(tk.END, text)
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    def run_command(self, label, cmd):
        def worker():
            try:
                self.status_var.set(f"Status: running «{label}»...")
                self.append_log(f"\n$ {' '.join(cmd)}\n", tag="cmd")

                proc = subprocess.Popen(
                    cmd,
                    cwd=WORKDIR,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )

                for line in proc.stdout:
                    self.append_log(line)

                ret = proc.wait()
                if ret == 0:
                    self.append_log(f"[OK] {label} finished successfully.\n", tag="ok")
                else:
                    self.append_log(
                        f"[ERROR] {label} exited with code {ret}.\n", tag="err"
                    )
                    messagebox.showerror(
                        "Error",
                        f"Command '{' '.join(cmd)}' exited with code {ret}.\n"
                        "See log for details.",
                    )
            except FileNotFoundError as e:
                self.append_log(f"[ERROR] {e}\n", tag="err")
                messagebox.showerror(
                    "Error",
                    f"Failed to run command: {e}\n"
                    "Do you have 'make' and 'docker' installed?",
                )
            finally:
                self.status_var.set("Status: idle")

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    if not os.path.isdir(WORKDIR):
        print(f"Error: WORKDIR '{WORKDIR}' does not exist.")
    else:
        app = DockerControlApp()
        app.mainloop()
