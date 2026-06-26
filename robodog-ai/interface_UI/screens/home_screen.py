from kivy.uix.screenmanager import Screen
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.graphics import Color, RoundedRectangle
from kivy.metrics import dp


class MenuCard(BoxLayout):
    """A tappable action card with an icon label and description."""

    def __init__(self, title, description, icon, target_screen, screen_manager=None, **kwargs):
        super().__init__(**kwargs)
        self.orientation = "vertical"
        self.padding = dp(12)
        self.spacing = dp(4)
        self.size_hint = (1, None)
        self.height = dp(100)
        self._sm = screen_manager
        self._target = target_screen

        with self.canvas.before:
            Color(0.13, 0.13, 0.15, 1)
            self._rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[dp(12)])

        self.bind(pos=self._update_rect, size=self._update_rect)

        self.add_widget(Label(
            text=icon,
            font_size=dp(26),
            size_hint=(1, None),
            height=dp(32),
            color=(0.4, 0.7, 1, 1),
        ))
        self.add_widget(Label(
            text=title,
            font_size=dp(14),
            bold=True,
            size_hint=(1, None),
            height=dp(20),
            color=(1, 1, 1, 1),
        ))
        self.add_widget(Label(
            text=description,
            font_size=dp(11),
            size_hint=(1, None),
            height=dp(16),
            color=(0.6, 0.6, 0.6, 1),
        ))

        touch_btn = Button(
            background_color=(0, 0, 0, 0),
            size_hint=(1, 1),
            pos_hint={"x": 0, "y": 0},
        )
        touch_btn.bind(on_release=self._navigate)
        self.add_widget(touch_btn)

    def _update_rect(self, *_):
        self._rect.pos = self.pos
        self._rect.size = self.size

    def set_screen_manager(self, screen_manager):
        self._sm = screen_manager

    def _navigate(self, *_):
        if self._sm and self._target:
            self._sm.current = self._target


class HomeScreen(Screen):
    """
    Main menu screen for the Robodog AI delivery system.

    Screens navigated from here:
        - delivery_form  → new delivery request
        - status_screen  → live robot tracking
        - qr_screen      → QR code scanner for pickup confirmation
        - countdown_screen → ETA / arrival timer
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._cards = []
        self._build_ui()

    def _build_ui(self):
        root = BoxLayout(orientation="vertical", padding=dp(20), spacing=dp(16))

        # ── Header ──────────────────────────────────────────────────────────
        header = BoxLayout(orientation="horizontal", size_hint=(1, None), height=dp(56))
        header.add_widget(Label(
            text="Robodog AI",
            font_size=dp(20),
            bold=True,
            color=(1, 1, 1, 1),
            halign="left",
            text_size=(None, None),
            size_hint=(1, 1),
        ))
        header.add_widget(Label(
            text="🟢 Online",
            font_size=dp(12),
            color=(0.3, 0.9, 0.5, 1),
            size_hint=(None, 1),
            width=dp(80),
        ))
        root.add_widget(header)

        # ── Stat row ─────────────────────────────────────────────────────────
        stats = GridLayout(cols=3, size_hint=(1, None), height=dp(64), spacing=dp(8))
        for label, value in [("Deliveries", "4"), ("Floor", "3 / 8"), ("Queue", "2")]:
            col = BoxLayout(orientation="vertical")
            col.add_widget(Label(text=value, font_size=dp(18), bold=True, color=(1, 1, 1, 1)))
            col.add_widget(Label(text=label, font_size=dp(11), color=(0.55, 0.55, 0.55, 1)))
            stats.add_widget(col)
        root.add_widget(stats)

        # ── Section label ─────────────────────────────────────────────────────
        root.add_widget(Label(
            text="ACTIONS",
            font_size=dp(10),
            color=(0.45, 0.45, 0.45, 1),
            size_hint=(1, None),
            height=dp(20),
            halign="left",
        ))

        # ── Action card grid ──────────────────────────────────────────────────
        grid = GridLayout(cols=2, spacing=dp(10), size_hint=(1, None), height=dp(220))

        actions = [
            ("New delivery",  "Send a package",         "📦", "delivery_form"),
            ("Track robot",   "Live position",          "📍", "status_screen"),
            ("Scan QR",       "Confirm pickup",         "⬛", "qr_screen"),
            ("ETA timer",     "Arrival countdown",      "⏱️", "countdown_screen"),
        ]

        for title, desc, icon, target in actions:
            card = MenuCard(
                title=title,
                description=desc,
                icon=icon,
                target_screen=target,
                screen_manager=self.manager if self.manager else None,
            )
            self._cards.append(card)
            grid.add_widget(card)

        root.add_widget(grid)

        # ── Voice command button ──────────────────────────────────────────────
        voice_btn = Button(
            text="🎙  Voice command",
            font_size=dp(14),
            size_hint=(1, None),
            height=dp(48),
            background_color=(0.18, 0.18, 0.22, 1),
            color=(1, 1, 1, 1),
        )
        voice_btn.bind(on_release=self._on_voice_command)
        root.add_widget(voice_btn)

        # ── Start delivery button ─────────────────────────────────────────────
        start_btn = Button(
            text="Start delivery",
            font_size=dp(15),
            bold=True,
            size_hint=(1, None),
            height=dp(52),
            background_color=(0.2, 0.5, 1, 1),
            color=(1, 1, 1, 1),
        )
        start_btn.bind(on_release=self._go_to_delivery_form)
        root.add_widget(start_btn)

        self.add_widget(root)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _go_to_delivery_form(self, *_):
        if self.manager:
            self.manager.current = "delivery_form"

    def _on_voice_command(self, *_):
        """
        Hook this up to your Speech-to-Text module.
        Example: call core/speech_handler.py or interfaces/stt_listener.py here.
        """
        print("[HomeScreen] Voice command triggered — connect STT here")
        # TODO: from core.stt import listen; listen(callback=self._handle_voice_result)

    def on_enter(self):
        """Called every time this screen becomes active — refresh stats here."""
        for card in self._cards:
            card.set_screen_manager(self.manager)
        print("[HomeScreen] Screen entered")
        # TODO: pull live battery %, floor, queue count from hardware/robot_state.py
