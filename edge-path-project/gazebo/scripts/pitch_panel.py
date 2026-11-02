"""Read-only Finnish presenter panel. Losing the panel cannot affect control."""

import json
import multiprocessing
import socket
import time


def show_panel(channel):
    try:
        import tkinter as tk
        root = tk.Tk()
    except (ImportError, RuntimeError) as error:
        print(f"Tilannepaneeli ei käynnistynyt ({error}). Tilat näkyvät terminaalissa.", flush=True)
        return
    except Exception as error:
        # Tk raises TclError when the display server is unavailable.
        print(f"Tilannepaneelin näyttö ei ole käytettävissä ({error}).", flush=True)
        return
    root.title("CAMINO · live")
    root.configure(bg="#101e2b")
    root.attributes("-topmost", True)
    root.geometry("-16+56")
    fields = []
    for text, color, size in (("CAMINO   /   RUSKOTUNTURI", "#9fb4c3", 11), ("Valmistellaan", "#ffffff", 18),
                              ("A", "#61e4d0", 16), ("B", "#ccacff", 16), ("", "#f5d378", 13)):
        label = tk.Label(root, text=text, fg=color, bg="#101e2b", font=("Sans", size, "bold"),
                         anchor="w", padx=12, pady=3)
        label.pack(fill="x")
        fields.append(label)
    channel.setblocking(False)
    last_live = None

    def poll():
        nonlocal last_live
        while True:
            try:
                payload = channel.recv(4096)
            except BlockingIOError:
                break
            if not payload:
                root.destroy()
                return
            state = json.loads(payload)
            fields[1].configure(text=state["status"])
            fields[2].configure(text="A   " + state["a"])
            fields[3].configure(text="B   " + state["b"])
            fields[4].configure(text=state["can"])
            last_live = time.monotonic() if state["live"] else None
        if last_live is not None and time.monotonic() - last_live > 1:
            fields[1].configure(text="Odottaa tilatietoa")
            fields[4].configure(text="Yhteystieto vanhentunut")
        root.after(100, poll)

    poll()
    root.mainloop()


class PitchPanel:
    def __init__(self):
        self.channel = None
        self.process = None
        self.labels = None

    def open(self):
        self.channel, receiver = socket.socketpair(type=socket.SOCK_DGRAM)
        self.channel.setblocking(False)
        self.process = multiprocessing.get_context("spawn").Process(target=show_panel, args=(receiver,), daemon=True)
        self.process.start()
        receiver.close()

    def update(self, a, b, can, status, live):
        labels = (a, b, can.split(" · laskuri")[0], live)
        if labels != self.labels:
            print(f"{status}  |  A: {a}  |  B: {b}  |  {can}", flush=True)
            self.labels = labels
        if self.channel:
            data = dict(a=a, b=b, can=can, status=status, live=live)
            try:
                self.channel.send(json.dumps(data, ensure_ascii=False).encode())
            except OSError:
                # A closed or slow optional window must never delay the watchdog.
                pass

    def close(self):
        if self.channel:
            self.channel.close()
        if self.process:
            self.process.terminate()
            self.process.join(timeout=2)
