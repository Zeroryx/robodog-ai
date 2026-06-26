from kivy.app import App
from kivy.config import Config
from kivy.core.window import Window
from kivy.metrics import dp
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.screenmanager import Screen, ScreenManager

Config.set("graphics", "width", "390")
Config.set("graphics", "height", "844")
Window.clearcolor = (0.07, 0.07, 0.09, 1)

try:
    from interface_UI.screens.home_screen import HomeScreen
except ImportError:
    from screens.home_screen import HomeScreen


class PlaceholderScreen(Screen):
    def __init__(self, title, **kwargs):
        super().__init__(**kwargs)
        layout = BoxLayout(orientation="vertical", padding=dp(24), spacing=dp(16))
        layout.add_widget(Label(text=title, font_size=dp(24), bold=True, color=(1, 1, 1, 1)))
        layout.add_widget(Label(text="This screen is ready for the next UI step.", color=(0.75, 0.75, 0.75, 1)))

        back_btn = Button(text="Back to Home", size_hint=(1, None), height=dp(48))
        back_btn.bind(on_release=self._go_home)
        layout.add_widget(back_btn)
        self.add_widget(layout)

    def _go_home(self, *_):
        if self.manager:
            self.manager.current = "home"


class RobodogApp(App):
    def build(self):
        sm = ScreenManager()
        home_screen = HomeScreen(name="home")
        sm.add_widget(home_screen)
        sm.add_widget(PlaceholderScreen("Delivery Form", name="delivery_form"))
        sm.add_widget(PlaceholderScreen("Robot Status", name="status_screen"))
        sm.add_widget(PlaceholderScreen("QR Scanner", name="qr_screen"))
        sm.add_widget(PlaceholderScreen("Countdown", name="countdown_screen"))
        return sm


if __name__ == "__main__":
    RobodogApp().run()
