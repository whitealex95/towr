from kivy.app import App
from kivy.lang import Builder
from kivy.graphics import Line, Color, InstructionGroup
from kivy.uix.widget import Widget


class MyWidget(Widget):

    undolist = []
    objects = []
    drawing = False

    def on_touch_up(self, touch):
        self.drawing = False

    def on_touch_move(self, touch):
        if self.drawing:
            self.points.append(touch.pos)
            self.obj.children[-1].points = self.points
        else:
            self.drawing = True
            self.points = [touch.pos]
            self.obj = InstructionGroup()
            self.obj.add(Color(1,0,0))
            self.obj.add(Line())
            self.objects.append(self.obj)
            self.canvas.add(self.obj)


    def undo(self):
        item = self.objects.pop(-1)
        self.undolist.append(item)
        self.canvas.remove(item)

    def redo(self):
        item = self.undolist.pop(-1)
        self.objects.append(item)
        self.canvas.add(item)


KV = """

BoxLayout:
    MyWidget:
        id: widget
    Button:
        text: "undo"
        on_release:
            widget.undo()
    Button:
        text: "redo"
        on_release:
            widget.redo()


"""


class MyApp(App):

    def build(self):
        root = Builder.load_string(KV)
        return root

MyApp().run()
