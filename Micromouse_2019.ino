#include <Maze.h>
#include "Micromouse.hpp"

const int hardResetButton = PB1;
const int softResetButton = PB2;

Maze maze( 16, 16 );
Micromouse mouse( maze, maze.height - 1, 0 );

void setup() {
  pinMode( hardResetButton, INPUT );
  pinMode( softResetButton, INPUT );
}

void loop() {
  
  if( !mouse.isDone() ) {
    /* move mouse to the next most optimal cell */
    mouse.exploreNextCell();
  }

  if( digitalRead(hardResetButton) == HIGH ) {
    /* erases all maze memory that the mouse discovered */
    mouse.start();
  }
  else if( digitalRead(softResetButton) == HIGH ) {
    /* resets the mouse position to initial location */
    mouse.restart();
  }
}
