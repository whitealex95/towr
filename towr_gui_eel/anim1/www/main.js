// setup canvas

const canvas = document.querySelector('canvas');
const ctx = canvas.getContext('2d');
ctx.filter = 'blur(4px)';

let width = canvas.width = window.innerWidth;
let height = canvas.height = window.innerHeight;

function initWindow() {
  width = canvas.width = window.innerWidth;
  height = canvas.height = window.innerHeight;
}

function matmul(m1, m2) {
  var result = [];
  for (var i = 0; i < m1.length; i++) {
      result[i] = [];
      for (var j = 0; j < m2[0].length; j++) {
          var sum = 0;
          for (var k = 0; k < m1[0].length; k++) {
              sum += m1[i][k] * m2[k][j];
          }
          result[i][j] = sum;
      }
  }
  return result;
}

class Grid {
  min_grid_size = 45;
  grid_levels = [0.1, 0.125, 0.2, 1/35, 0.5, 1, 2, 3, 5, 8, 10];

  constructor(xmin, xmax, ymin, ymax, xn, yn, v) {
    this.xmin = xmin;
    this.xmax = xmax;
    this.ymin = ymin;
    this.ymax = ymax;
    this.adjust_minmax();
    this.xn = 2 * xn;
    this.yn = 2 * yn;
    this.v = v;

    this.origin_x = width/2;
    this.origin_y = height *2 /3;
    this.ratio_c2g = null;
  }
  adjust_minmax(){
    let adjust_ratio = ((this.xmax - this.xmin) / (this.ymax - this.ymin)) / (width/height)
    if(adjust_ratio > 1){  // our height can contain more coordinate
      this.ymax *= adjust_ratio;
      this.ymin *= adjust_ratio;
    } else {
      this.xmax *= 1/adjust_ratio;
      this.xmin *= 1/adjust_ratio;
    }
  }
  draw() {
    // this.draw_axis();
    this.draw_grid();
  }
  
  setGridSize(){ // remove xmin, xmax
    this.grid_level = null;
    this.grid_size = null;
    for(let i=this.grid_levels.length-1; i>=0; i--){
      let level = this.grid_levels[i];
      let gs_x = width / ((this.xmax-this.xmin)/level);
      let gs_y = height / ((this.ymax-this.ymin)/level);
      let gs = Math.min(gs_x, gs_y);
      if(gs >= this.min_grid_size){
        this.grid_level = level;
        this.grid_size = gs;
      }
    }
    this.ratio_c2g = this.grid_level/this.grid_size;
    this.ratio_g2c = 1/this.ratio_c2g;
    // console.log(this.grid_level, this.grid_size)
  }

  draw_axis() {
    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#000000";
    // x-axis
    ctx.moveTo(0, this.origin_y);
    ctx.lineTo(width, this.origin_y);
    // y-axis
    ctx.moveTo(this.origin_x, 0);
    ctx.lineTo(this.origin_x, height);
    ctx.stroke();
  }
  draw_grid() {
    this.draw_axis();
    this.setGridSize();
    let grid_size = this.grid_size; // 25 pixel fixed size grid;
    let grid_level = this.grid_level;
    if(grid_size == null) {
      return;
    }

    // draw grid parallel to x-axis
    for(let i=1; this.origin_y + grid_size*i < height; i++){
      // draw line above x-axis
      ctx.beginPath();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#a9a9a9";
      ctx.moveTo(0, this.origin_y + grid_size*i);
      ctx.lineTo(width, this.origin_y + grid_size*i);
      ctx.stroke();
    }
    for(let i=1; this.origin_y - grid_size*i > 0; i++){
      // draw line above x-axis
      ctx.beginPath();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#a9a9a9";
      ctx.moveTo(0, this.origin_y - grid_size*i);
      ctx.lineTo(width, this.origin_y - grid_size*i);
      ctx.stroke();
    }

    // draw grid parallel to y-axis
    for(let i=1; this.origin_x + grid_size*i < width; i++){
      // draw line above x-axis
      ctx.beginPath();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#a9a9a9";
      ctx.moveTo(this.origin_x + grid_size*i, 0);
      ctx.lineTo(this.origin_x + grid_size*i, height);
      ctx.stroke();
    }
    for(let i=1; this.origin_x - grid_size*i > 0; i++){
      // draw line above x-axis
      ctx.beginPath();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#a9a9a9";
      ctx.moveTo(this.origin_x - grid_size*i, 0);
      ctx.lineTo(this.origin_x - grid_size*i, height);
      ctx.stroke();
    }

    // draw ticks along x-axis
    let text;
    for(let i=0; this.origin_x + grid_size*i < width; i++){
      ctx.beginPath();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#000000";

      // Draw a tick mark 6px long (-3 to 3)
      ctx.moveTo(this.origin_x + grid_size*i, this.origin_y-6);
      ctx.lineTo(this.origin_x + grid_size*i, this.origin_y+6);
      ctx.stroke();

      // Text value at that point
      ctx.font = '15px Arial';
      ctx.textAlign = 'start';
      ctx.fillStyle = "#696969";
      text = (grid_level*i)
      if(grid_level < 1) text = text.toPrecision(2);
      ctx.fillText(text, this.origin_x + grid_size*i+2, this.origin_y+20);
    } 
    for(let i=1; this.origin_x - grid_size*i > 0; i++){
      ctx.beginPath();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#000000";

      // Draw a tick mark 6px long (-3 to 3)
      ctx.moveTo(this.origin_x - grid_size*i, this.origin_y-6);
      ctx.lineTo(this.origin_x - grid_size*i, this.origin_y+6);
      ctx.stroke();

      // Text value at that point
      ctx.font = '15px Arial';
      ctx.textAlign = 'start';
      ctx.fillStyle = "#696969";
      text = (-grid_level*i)
      if(grid_level < 1) text = text.toPrecision(2);
      ctx.fillText(text, this.origin_x - grid_size*i+2, this.origin_y+20);
    } 
    // draw ticks along y-axis
    for(let i=1; this.origin_y + grid_size*i < height; i++){
      ctx.beginPath();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#000000";

      // Draw a tick mark 6px long (-3 to 3)
      ctx.moveTo(this.origin_x-6, this.origin_y + grid_size*i);
      ctx.lineTo(this.origin_x+6, this.origin_y + grid_size*i);
      ctx.stroke();

      // Text value at that point
      ctx.font = '15px Arial';
      ctx.textAlign = 'end';
      ctx.fillStyle = "#696969";
      text = (-grid_level*i)
      if(grid_level < 1) text = text.toPrecision(2);
      ctx.fillText(text, this.origin_x-10, this.origin_y + grid_size*i+15);
    } 
    for(let i=1; this.origin_y - grid_size*i > 0; i++){
      ctx.beginPath();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#000000";

      // Draw a tick mark 6px long (-3 to 3)
      ctx.moveTo(this.origin_x-6, this.origin_y- grid_size*i);
      ctx.lineTo(this.origin_x+6, this.origin_y- grid_size*i);
      ctx.stroke();

      // Text value at that point
      ctx.font = '15px Arial';
      ctx.textAlign = 'end';
      ctx.fillStyle = "#696969";
      text = (grid_level*i)
      if(grid_level < 1) text = text.toPrecision(2);
      ctx.fillText(text , this.origin_x-10, this.origin_y - grid_size*i+15);
    } 
    ctx.lineWidth=1;
  }
  getCanvasCoord(x, y) {
    let x_canv = x * this.ratio_g2c + this.origin_x;
    let y_canv = -y * this.ratio_g2c + this.origin_y;
    return [x_canv, y_canv];
  }

  getGridCoord(x_canv, y_canv) {
    let x = (x_canv - this.origin_x) * this.ratio_c2g;
    let y = -(y_canv - this.origin_y) * this.ratio_c2g;
    return [x,y];
  }
}
class ControlPoint {
  constructor(xc, yc, parent=null, child=null) {
    this.xc = xc;
    this.yc = yc;
    [this.x, this.y] = grid.getGridCoord(xc, yc);
    this.color = 'black';
    this.lineColor = 'black';
    this.size = 10;
    this.parent = parent;
    this.child = child;
    // this.selected = selected;
  }
  remove() {
    if(this.child) this.child.parent = this.parent;
    if(this.parent) this.parent.child = this.child;
  }
  addChild(cp) {
    this.child = cp;
  }
  computeCoord() {
    [this.xc, this.yc] = grid.getCanvasCoord(this.x, this.y);
  }
  computeGrid() {
    [this.x, this.y] = grid.getGridCoord(this.xc, this.yc);
  }
  draw() {
    ctx.beginPath();  // state that we want to draw
    ctx.strokeStyle = this.color;
    ctx.lineWidth = 3;
    ctx.arc(this.xc, this.yc, this.size, 0, 2 * Math.PI);
    ctx.stroke();  
  }
  draw_head() {
    ctx.beginPath();  // state that we want to draw
    ctx.strokeStyle = 'red';
    ctx.lineWidth = 3;
    ctx.arc(this.xc, this.yc, this.size, 0, 2 * Math.PI);
    ctx.stroke();  
  }
  draw_selected() {
    ctx.beginPath();  // state that we want to draw
    ctx.fillStyle = this.color;
    ctx.arc(this.xc, this.yc, this.size, 0, 2 * Math.PI);
    ctx.fill();
  }
  draw_selected_head() {
    ctx.beginPath();  // state that we want to draw
    ctx.fillStyle = 'red';
    ctx.lineWidth = 3;
    ctx.arc(this.xc, this.yc, this.size, 0, 2 * Math.PI);
    ctx.fill();
  }

  draw_line(color=null) {
    if(this.child == null) return;
    ctx.beginPath();
    ctx.strokeStyle = color? color: this.lineColor;
    ctx.moveTo(this.xc, this.yc);
    ctx.lineTo(this.child.xc, this.child.yc);
    ctx.stroke();
  }
}

class ControlPointList {
  constructor(head = null) {
    this.head = head;

    this.curve_state = {
      'line': true,
      'bezier': false,
      'bspline': false,
      'crspline': true,
    }
    this.curve_color = {
      'line': 'orange',
      'bezier': 'purple',
      'bspline': 'green',
      'crspline': 'blue',
    }
  }
  computeCoords() {
    let cp = this.head;
    while(cp){
      cp.computeCoord();
      cp = cp.child;
    }
  }
  computeGrids() {
    let cp = this.head;
    while(cp){
      cp.computeGrid();
      cp = cp.child;
    }
  }
  draw() {
    this.draw_points();
    if(this.curve_state['line'])
      this.draw_line_interpolation();
    if(this.curve_state['bezier'])
      this.draw_bezier_interpolation();
    if(this.curve_state['bspline'])
      this.draw_bspline_interpolation();
    if(this.curve_state['crspline'])
      this.draw_crspline_interpolation();
  }
  draw_points() {
    let cp = this.head;
    while(cp){
      if(cp==controlPointList.head) {
        if(cp == current_cp) cp.draw_selected_head();
        else cp.draw_head();
      } else if(cp==current_cp) cp.draw_selected();
      else cp.draw();
      cp = cp.child;
    }
  }
  draw_line_interpolation() {
    let cp = this.head;
    while(cp){
      cp.draw_line(this.curve_color['line']);
      cp = cp.child;
    }
  
  }
  draw_bezier_interpolation() {
    let p0, p1, p2, p3;
    let x0, y0, x1, y1;
    let linspace = 1/100;

    for(let i = 0; i < n_cp/3-1; i++) {
      if(i==0) p0 = this.head;
      else p0 = p3;
      p1 = p0.child;
      p2 = p1.child;
      p3 = p2.child;
      
      [x0, y0] = [p0.xc, p0.yc];
      for(let t=linspace; t<1+linspace; t+=linspace){
        ctx.beginPath();
        ctx.strokeStyle = this.curve_color['bezier'];
        x1 = (1-t)**3 * p0.xc + 3*t*(1-t)**2 * p1.xc + 3*t**2*(1-t) * p2.xc + t**3 * p3.xc;
        y1 = (1-t)**3 * p0.yc + 3*t*(1-t)**2 * p1.yc + 3*t**2*(1-t) * p2.yc + t**3 * p3.yc;
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
        [x0,y0] = [x1,y1];
      }
    }
  }

  draw_spline_interpolation(spline_at_t, color) {
    let p0, p1, p2, p3;
    let x0, y0, x1, y1;
    let linspace = 1/100;

    let points = [];
    for(let i = 0; i < (n_cp-3); i++) {
      if(i==0) p0 = this.head;
      else p0 = p1;
      p1 = p0.child;
      p2 = p1.child;
      p3 = p2.child;
      
      [x0, y0] = [p0.xc, p0.yc];
      for(let t=0; t<1; t+=linspace){
        [x1,y1] = spline_at_t(p0, p1, p2, p3, t);
        points.push([x1,y1]);
      }
    }
    if (points.length > 0)
      [x0,y0] = points[0];
    for(let i=1;i < points.length; i++){
      ctx.beginPath();
      ctx.strokeStyle = color;
      [x1,y1] = points[i]
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
      [x0,y0] = [x1,y1];

    }
  }
  draw_bspline_interpolation() {
    this.draw_spline_interpolation(this.bspline_at_t,
      this.curve_color['bspline']);
  }
  draw_crspline_interpolation() {
    this.draw_spline_interpolation(this.crspline_at_t,
      this.curve_color['crspline']);
  }
  bspline_at_t(p0, p1, p2, p3, t){
    let M = [[-1/6, 3/6, -3/6, 1/6],
          [3/6, -6/6, 3/6, 0/6],
          [-3/6, 0/6, 3/6, 0/6],
          [1/6, 4/6, 1/6, 0/6]];
    let Q = matmul([[t**3, t**2, t, 1]], M)
    Q = matmul(Q, [[p0.xc, p0.yc],
                  [p1.xc, p1.yc],
                  [p2.xc, p2.yc],
                  [p3.xc, p3.yc]])
    return Q[0];
  }
  crspline_at_t(p0, p1, p2, p3, t){
    let M = [[-1/2, 3/2, -3/2, 1/2],
          [2/2, -5/2, 4/2, -1/2],
          [-1/2, 0/2, 1/2, 0/2],
          [0/2, 2/2, 0/2, 0/2]];
    let Q = matmul([[t**3, t**2, t, 1]], M)
    Q = matmul(Q, [[p0.xc, p0.yc],
                  [p1.xc, p1.yc],
                  [p2.xc, p2.yc],
                  [p3.xc, p3.yc]])
    return Q[0];

  }
  size() {
    let count = 0; 
    let node = this.head;
    while(node) {
      count++;
      node = node.child;
    }
    return count;
  }
  clear() {
    this.head = null;
  }
  getLast() {
    let lastNode = this.head;
    if (lastNode) {
      while (lastNode.child) {
        lastNode = lastNode.child;
      }
    }
    return lastNode;
  }
  getFirst() {
    return this.head;
  }
  remove(node) {
    if (node.parent) {
      node.parent.child = node.child;
    }
    if (node.child) {
      node.child.parent = node.parent;
    }
    if (node == this.head) {
      this.head = node.child;
    }
  }
}

const EMPTY = 0;
const NEW_CLICK = 1;
const SEL_CLICK = 2;
const DRAG_CLICK = 3;
const SEL_THRESHOLD = 20;

let click_status = EMPTY;
let current_cp = null;
let parent_cp = null;

let controlPoints = [];
let controlPointList = new ControlPointList();
let n_cp = 0;

function selectControlPoint(x, y) {
  // optimize here
  let neaerst_d = SEL_THRESHOLD;
  nearest_cp = null;
  cp = controlPointList.head;
  while(cp){
    d = ((cp.xc - x)**2 + (cp.yc - y)**2)**0.5;
    if (d < neaerst_d){
      nearest_cp = cp;
      neaerst_d = d;
    }
    cp = cp.child;
  }
  return nearest_cp;
}

let grid = new Grid(-7, 7, -7, 7, 8, 8, 10);

function init() {
  initWindow();
  ctx.fillStyle = '#f0f9f9';
  ctx.fillRect(0,0,width, height);
  grid.draw();
}
function updateCount(){
  document.getElementById('counter').textContent = "Number of ControlPoints: " 
  + n_cp;
}
function updateScreen() {
  init();
  controlPointList.draw();
  updateCount();
}


let lastPoint = null;
canvas.onmousedown = function(e) {
  console.log(e.which);
  if (e.which == 2) {
    // middle clicking can should drag grid
    click_status = DRAG_CLICK;
    lastPoint = [e.offsetX, e.offsetY];
    return;
  }
  cp = selectControlPoint(e.offsetX, e.offsetY);
  if (cp == null) {
    click_status = NEW_CLICK;
    parent_cp = current_cp;
    child_cp = parent_cp ? parent_cp.child : null;
    current_cp = new ControlPoint(e.offsetX, e.offsetY, parent_cp, child_cp);
    parent_cp ? parent_cp.addChild(current_cp) : null;
    child_cp ? child_cp.parent = current_cp : null;
    // child_cp ? child_cp.parent = parent_cp : null;
    n_cp += 1;
    if (n_cp == 1) {
      controlPointList = new ControlPointList(current_cp);
    }
  } else {
    click_status = SEL_CLICK;
    current_cp = cp;
  }
  lastPoint = [e.offsetX, e.offsetY];
  updateScreen();
}
canvas.onmouseup = function(e) {
  console.log(e)
  click_status = EMPTY;
  controlPointList.computeGrids();
  updateScreen()
}
canvas.onmousemove = function(e) {
  console.log('m: ', e.which)
  if(click_status == EMPTY) {
    return;
  } else if(click_status == DRAG_CLICK) {
    // drag
    let dx = e.offsetX - lastPoint[0];
    let dy = e.offsetY - lastPoint[1];

    grid.origin_x += dx;
    grid.origin_y += dy;

    dx *= grid.ratio_c2g;
    dy *= grid.ratio_c2g;
    
    grid.xmax -= dx;
    grid.xmin -= dx;
    grid.ymax += dy;
    grid.ymin += dy;

    controlPointList.computeCoords();
  } else {
    current_cp.xc += e.offsetX - lastPoint[0];
    current_cp.yc += e.offsetY - lastPoint[1];
  }
  lastPoint = [e.offsetX, e.offsetY];
  updateScreen();
}

window.onkeydown = function(e) {
  // console.log(e.key)
  if (e.key == 'x') {
    if (current_cp == null) return;
    // remove
    if(current_cp == controlPointList.head){
      controlPointList.head = current_cp.child;
      parent_cp = current_cp.parent;
      current_cp.remove();
      current_cp = controlPointList.head;
    } else {
      parent_cp = current_cp.parent;
      current_cp.remove();
      current_cp = parent_cp;  
    }
    n_cp--;
  }
  updateScreen();
}
canvas.onwheel = function(e) {
  // console.log(e.x, e.y, e.deltaY > 0 ? 'scroll out' : 'scroll in');
  let scale = 1;
  if(e.deltaY > 0) scale = 1.1;
  if(e.deltaY < 0) scale = 1/1.1;

  grid.xmax = e.x + (grid.xmax - e.x) * scale;
  grid.xmin = e.x + (grid.xmin - e.x) * scale;
  grid.ymax = e.y + (grid.ymax - e.y) * scale;
  grid.ymin = e.y + (grid.ymin - e.y) * scale;
  grid.origin_x = e.x + (grid.origin_x - e.x) * scale;
  grid.origin_y = e.y + (grid.origin_y - e.y) * scale;

  controlPointList.computeCoords();
  updateScreen();
}
document.getElementsByTagName('body')[0].onresize = function() {
  updateScreen();
}

curveTypes = ['line', 'bezier', 'bspline', 'crspline'];
for(let i=0; i<curveTypes.length; i++){
  let curveType = curveTypes[i];
  let checkbox = document.getElementById(curveType);
  controlPointList.curve_state[curveType] = checkbox.checked;
  checkbox.onclick = (e)=>{
    controlPointList.curve_state[curveType] = checkbox.checked;
    updateScreen();
  }
}
init();

let example = function(i) {
  let Vx_eq, Vy_eq, dt;
  if (i==1) {
    Vx_eq = "-2*y"
    Vy_eq = "2*x"
    dt = 0.4
  } else if (i==2) {
    Vx_eq = "-2*y"
    Vy_eq = "2*x"
    dt = 0.1
  } else if (i==3) {
    Vx_eq = "x+y"
    Vy_eq = "x-y"
    dt = 0.1
  }
  vf.update_Vx(Vx_eq)
  vf.update_Vy(Vy_eq)
  ds.dt = dt;
  ds.compute();
  updateScreen();
  Vx_input.value = vf.Vx_eq;
  Vy_input.value = vf.Vy_eq;
  UIstepSize.innerText = dt;
  UIstepSizeSlider.value = 100 + Math.log10(dt) * 100 / 3
}
