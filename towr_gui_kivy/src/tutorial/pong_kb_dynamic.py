# Pingpong Tutorial
# https://kivy.org/doc/stable/tutorials/pong.html
# add keyboard for here: https://stackoverflow.com/questions/17280341/how-do-you-check-for-keyboard-events-with-kivy
from kivy.core.window import Window

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty
from kivy.vector import Vector

from kivy.clock import Clock
from random import randint
# Q. Widget vs App vs Layout?

class PongPaddle(Widget):
  score = NumericProperty(0)

  def bounce_ball(self, ball):
    if (self.collide_widget(ball)):
      vx, vy = ball.velocity
      # what is offset?
      offset = (ball.center_y - self.center_y) / (self.height / 2)
      bounced = Vector(-1 * vx, vy)
      vel = bounced * 1.1
      ball.velocity = vel.x, vel.y + offset


class PongGame(Widget):
  ball = ObjectProperty(None)
  player1 = ObjectProperty(None)
  player2 = ObjectProperty(None)

  def __init__(self, **kwargs):
    super(PongGame, self).__init__(**kwargs)
    self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
    self._keyboard.bind(on_key_down=self._on_keyboard_down)
    self.balls = [self.ball]
    # self.balls = []
    # self.balls.append(self.ball)

  def start(self):
    for ball in self.balls:
      ball.serve(self.center)
      
  
  def update(self, dt):
    for ball in self.balls:
      ball.move()

      # bounce ball if required
      self.player1.bounce_ball(ball)
      self.player2.bounce_ball(ball)

      # bounce off top and bottom
      if (ball.y < self.y) or (ball.top > self.top):
        ball.velocity_y *= -1
      
      if ball.x < self.x:
        self.player2.score += 1    
        ball.serve(pos=self.center, vel=(4,0))
      if ball.x > self.width:
        self.player1.score += 1
        ball.serve(pos=self.center, vel = (-4,0))
    
  def _keyboard_closed(self):
    self._keyboard.unbind(on_key_down=self._on_keyboard_down)
    self._keyboard = None
  
  def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
    if keycode[1] == 'w':
        self.player1.center_y += 10
    elif keycode[1] == 's':
        self.player1.center_y -= 10
    elif keycode[1] == 'up':
        self.player2.center_y += 10
    elif keycode[1] == 'down':
        self.player2.center_y -= 10
    elif keycode[1] == 'spacebar':
      self._add_ball()
    return True
  
  def _add_ball(self):
    newball = PongBall()
    newball.serve(self.center)
    self.balls.append(newball)
    self.add_widget(newball)

class PongApp(App):
  def build(self):
    game = PongGame()
    # game.serve_ball(Vector(4, 0).rotate(randint(0, 360)))
    game.start()
    Clock.schedule_interval(game.update, 1.0/60.0)
    return game


class PongBall(Widget):
  velocity_x = NumericProperty(0)
  velocity_y = NumericProperty(0)

  # enables use of ball.velocity as a short-hand like w.pos for w.x and w.y
  velocity = ReferenceListProperty(velocity_x, velocity_y)

  def move(self):
    self.pos = Vector(*self.velocity) + self.pos
    # print(self.pos) # self.pos? 가 뭔가요? vector 타입인가요? 흠흠 처음 init을 어떻게 하는 지 안 적ㅎ려있네

  def serve(self, pos, vel=(4,0)):
    self.center = pos
    self.velocity = vel

  

if __name__ == '__main__':
  PongApp().run()