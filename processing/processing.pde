import processing.net.*;

Client clientPython; 

PImage bg;
String map_name = "liu.png";
float x_pen = 0;
float y_pen = 0;
// Canvas size: will be corrected by the input background image size
int bg_width = 600;
int bg_height = 400;

void setup() {
  // size(225, 225); // default window size
  bg = loadImage(dataPath(map_name));
  bg_width = bg.width;
  bg_height = bg.height;
  windowResize(bg_width, bg_height);
  
  // Connect to the local machine at port 327.
  // This example will not run if you haven't
  // previously started a server on this port.
  // Make sure the address and the port are the
  // same ones in Python.
  clientPython = new Client(this, "127.0.0.1", 327); 
  clientPython.clear();
}

void draw() {
  background(bg);
  
  fill(255, 0, 0);
  ellipse(x_pen, y_pen, 10, 10);

}

void clientEvent(Client clientPython) {
  //*********************************************************
  //** Read in Proposed Pen Positions from Socket (START) ***
  //*********************************************************
  if (clientPython.active() == true && clientPython.available() > 0) {
    String inputString = clientPython.readStringUntil('\n');
    if (inputString != null) {
      try{
        String[] substring = inputString.split(" ");
        map_name = substring[0].trim();
        x_pen = Integer.valueOf(substring[1].trim());
        y_pen = Integer.valueOf(substring[2].trim());
      }
      catch(Exception e) {
        ;
      }
    }
  }
  //println(data);
  //*******************************************************
  //** Read in Proposed Pen Positions from Socket (END) ***
  //*******************************************************
}
