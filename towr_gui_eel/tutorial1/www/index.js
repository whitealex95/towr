function callPython() {
  eel.hello()
}

function addText(text) {
  var div = document.getElementById("content");
  div.innerHTML += "<br>" + text;
}
eel.expose(addText);