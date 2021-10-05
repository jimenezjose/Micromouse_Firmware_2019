/*******************************************************************************
                                                    Jose Jorge Jimenez-Olivas
                                                    Brandon Cramer
                                                    Chris Robles
                                                    Srinivas Venkatraman

                 University of California, San Diego
                      IEEE Micromouse Team 2019

File Name:       Micromouse.hpp 
Description:     Hardware stack implementation of miromouse navigation.
*******************************************************************************/
#ifndef MICROMOUSE_HPP
#define MICROMOUSE_HPP
#include "Mouse.hpp"

class Micromouse : public Mouse {
public:
  /*****************************************************************************
  % Constructor: Micromouse 
  % File:        Micromouse.hpp
  % Parameters:  maze - reference to maze data structure.
  %              row  - starting row for mouse.
  %              column - starting column for mouse.
  *****************************************************************************/
  Micromouse( Maze & maze, int row, int column ) : Mouse(maze, row, column) {}

  /*****************************************************************************
  % Routine Name: rotateTo
  % File:         Micromouse.hpp
  % Parameters:   cell - adjacent cell that mouse should be pointing forward to.
  % Description:  Rotates the mouse to have its front face point towards cell.
  % Return:       Nothing.
  *****************************************************************************/
  void rotateTo( MazeCell * cell ) {
    if( cell == nullptr ) return;
    if( column == cell->column ) {
      /* vertical deviation */
      if( row + 1 == cell->row ) orientation = SOUTH;
      else if( row - 1 == cell->row ) orientation = NORTH;
    }
    else if( row == cell->row ) {
      /* horizontal deviation */
      if( column + 1 == cell->column ) orientation = EAST;
      else if( column - 1 == cell->column ) orientation = WEST;
    }

    /* INSERT CODE HERE - harware rotateTo */
  }

  /*****************************************************************************
  % Routine Name: moveTo
  % File:         Micromouse.hpp
  % Parameters:   cell - adjacent cell to move to.
  % Description:  Moves the mouse to an adjacent cell.
  % Return:       Nothing.
  *****************************************************************************/
  void moveTo( MazeCell * cell ) {
    if( cell == nullptr ) return;
    row = cell->row;
    column = cell->column;

    /* INSERT CODE HERE - harware moveTo */
  }
 
  /*****************************************************************************
  % Routine Name: markNeighborWalls
  % File:         Micromouse.hpp 
  % Parameters:   cell - current cell.
  %               orientation - current orientation of mouse.
  % Description:  Adds walls to the maze given if the mouse detects walls on it 
  %               surroundings.
  % Return:       Nothing. 
  *****************************************************************************/
  void markNeighborWalls( MazeCell * cell, Orientation orientation ) {
    /* INSERT CODE HERE - hardware markNeighborWalls */
  }

};
#endif /* MICROMOUSE_HPP */
