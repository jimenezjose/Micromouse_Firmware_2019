/*******************************************************************************
                                                    Jose Jorge Jimenez-Olivas
                                                    Brandon Cramer
                                                    Chris Robles
                                                    Srinivas Venkatraman
                 University of California, San Diego
                      IEEE Micromouse Team 2019
File Name:       Micromouse_2019.hpp 
Description:     Main source file to run micromouse firmware.
*******************************************************************************/
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
