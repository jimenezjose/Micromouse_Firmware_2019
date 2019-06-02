/*******************************************************************************
                                                    Jose Jorge Jimenez-Olivas
                                                    Brandon Cramer
                                                    Chris Robles
                                                    Srinivas Venkatraman

                 University of California, San Diego
                      IEEE Micromouse Team 2019

File Name:       Mouse.h
Description:     Abstract Mouse interface with dependent hardware functions. 
*******************************************************************************/
#ifndef MOUSE_HPP
#define MOUSE_HPP

/* stm32 architecture or non arduino */
#if defined( ARDUINO_ARCH_STM32 ) || !defined( ARDUINO )
  #include <iostream>
  #include <utility>
  #include <vector>
  #include <array>
  #include <deque>
  #include <climits>
  #include "Maze/Maze.h"
  #include "utility/Orientation.hpp"
#else
  #error "board not supported." 
#endif

class Mouse {
public:
  Maze & maze;
  int row = 0;
  int column = 0;
  Orientation orientation = NORTH;
  std::vector<std::vector<bool>> visited;

  /*****************************************************************************
  % Constructor: Abstract Mouse with starting location - row, column
  % File:        Mouse.hpp
  % Parameters:  maze   - maze data structure.
  %              row    - starting row of mouse.
  %              column - starting column of mouse.
  *****************************************************************************/
  Mouse( Maze & maze, int row, int column ) : maze( maze ), 
      visited( maze.height, std::vector<bool>(maze.width, false) ) {
    
    if( maze.outOfBounds(row, column) ) {
      #ifndef ARDUINO
        std::cerr << "Mouse position: (" << row << ", " << column << ")" 
	          << "out of bounds." << std::endl;
      #endif
      row = column = 0;
    }
    
    if( maze.height != maze.width ) {
      #ifndef ARDUINO
        std::cerr << "unsupported: non-square dimension of maze: height x width"
	          << ": " << maze.height << " x " << maze.width << std::endl;
      #endif
    }

    this->row = row;
    this->column = column;
    origin = std::make_pair( row, column );
    start_position = std::make_pair( row, column );
    start();
  }

  /*****************************************************************************
  % Routine Name: markNeighborWalls
  % File:         Mouse.hpp
  % Parameters:   cell - current cell.
  %               orientation - current orientation of mouse.
  % Description:  Adds walls to the maze given if the mouse detects walls on it 
  %               surroundings.
  % Return:       Nothing. 
  *****************************************************************************/
  virtual void markNeighborWalls( MazeCell * cell, Orientation orientation) = 0;
  /*****************************************************************************
  % Routine Name: rotateTo
  % File:         Mouse.hpp
  % Parameters:   cell - adjacent cell that mouse should be pointing forward to.
  % Description:  Rotates the mouse to have its front face point towards cell.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void rotateTo( MazeCell * cell ) = 0;
  /*****************************************************************************
  % Routine Name: moveTo
  % File:         Mouse.hpp
  % Parameters:   cell - adjacent cell to move to.
  % Description:  Moves the mouse to an adjacent cell.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void moveTo( MazeCell * cell ) = 0;

  /*****************************************************************************
  % Routine Name: exploreNextCell
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Flood Fill Algorithm iteration.
  % Return:       true if mouse is in progress to get to target; false if mouse 
  %               is at target.
  *****************************************************************************/
  virtual bool exploreNextCell() final {
    if( explore_stack.empty() ) {
      /* mouse is at target. */
      done = true;
      trackSteps();

      if( mousePath.size() == previousPath.size() && isCompletePath(mousePath)){
        /* An optimal path was discovered */
        return false;
      }

      /* otherwise continue traversing maze */
      done = false;
      retreat();
      previousPath = mousePath;
      return false;
    }

    MazeCell * cell = explore_stack.back();
    explore_stack.pop_back(); 

    /* sensor surroundings */
    rotateTo( cell );
    moveTo( cell );
    setVisited( cell, true );
    markNeighborWalls( cell, orientation );
    /* notify other cells of new walls */
    callibrateDistances( cell );

    for( MazeCell * openNeighbor : cell->getNeighborList() ) {
      /* choose best adjacent open cell */
      if( openNeighbor->distance == cell->distance - 1 ) {
        /* hueristic to move closer to the target */
        explore_stack.push_back( openNeighbor );
	if( openNeighbor->distance == 0 ) {
	  explore_stack.push_back( openNeighbor );
	}
	return true;
      } 
      else if( openNeighbor->distance == 0 && !hasVisited( openNeighbor ) ) {
        /* visit all target nodes in quad-cell solution */
	explore_stack.push_back( openNeighbor );
        return true;
      }
    }
    return true;
  }

  /*****************************************************************************
  % Routine Name: start
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Clears the mouse memory then restarts the mouse.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void start() final {
    clearMemory();
    restart();
  }
  
  /*****************************************************************************
  % Routine Name: restart
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Resets the mouse's position the origin position that was 
  %               passed to the constructor.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void restart() final {
    /* restart starting position */
    start_position = std::make_pair( origin.first, origin.second );
    explore_stack.push_back( maze.at(origin.first, origin.second) );
    setMousePosition( origin );
    orientation = NORTH;
  }

  /*****************************************************************************
  % Routine Name: updateMazeCellDistances
  % File:         Mouse.hpp
  % Parameters:   target - cell that will have a distance of 0.
  % Description:  Update distance values for each cell in the maze given the target.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void updateMazeCellDistances( MazeCell * target ) final {
    if( target == nullptr ) return;
    std::deque<MazeCell *> q;

    for( MazeCell * cell : maze ) {
      /* reset visited values of all cells in the maze */
      cell->setVisited( false );
    }

    q.push_back( target );
    target->setVisited( true );
    target->distance = 0;

    while( !q.empty() ) {
      /* BFS traversal */
      MazeCell * cell = q.front();
      q.pop_front();

      for( MazeCell * openNeighbor : cell->getNeighborList() ) {
        /* update distance only to open neighbor of cell */
        if( openNeighbor->visited ) continue;
	q.push_back( openNeighbor );
	openNeighbor->setVisited( true );
	openNeighbor->distance = cell->distance + 1;
      }
    }
  }

  /*****************************************************************************
  % Routine Name: getMousePath
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Getter for the most optimal path the mouse has found so far.
  % Return:       List of cells. 
  *****************************************************************************/
  virtual std::vector<MazeCell *> getMousePath() final {
    return mousePath;
  }

  /*****************************************************************************
  % Routine Name: getNumberOfRuns 
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Getter for the number of times the mouse reached the target.
  % Return:       The number of runs made thus far.
  *****************************************************************************/
  virtual int getNumberOfRuns() final {
    return ( isDone() ) ? num_of_runs + 1 : num_of_runs;
  }

  /*****************************************************************************
  % Routine Name: isDone
  % File:         Mouse.hpp
  % Parameters:   Nothing.
  % Description:  Checks if the mouse has found the most optimal path to the 
  %               target.
  % Return:       true if the mouse has found the optimal path to the target,
  %               false otherwise.
  *****************************************************************************/
  virtual bool isDone() final {
    return done;
  }

private:
  std::vector<MazeCell *> explore_stack;
  std::vector<MazeCell *> mousePath;
  std::vector<MazeCell *> previousPath;
  std::pair<int, int> start_position;  
  std::pair<int, int> origin;
  int num_of_runs = 0;
  bool done = false;

  /*****************************************************************************
  % Routine Name: callibrateDistances
  % File:         Mouse.hpp
  % Parameters:   cell - current cell.
  % Description:  Floods the current cell and its adjacent cell distance value 
  %               towards the target; This fuction delegates to callibrate.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void callibrateDistances( MazeCell * cell ) final {
    if( cell == nullptr ) return;
    callibrate( cell );
    for( MazeCell * globalNeighbor : maze.getAdjacentCellList( cell ) ) {
      if( globalNeighbor->distance == 0 ) continue;
      callibrate( globalNeighbor );
    }
  }
  
  /*****************************************************************************
  % Routine Name: calibrate
  % File:         Mouse.hpp
  % Parameters:   cell - cell in need of distance validation.
  % Description:  Floods currenct cell such that there exist an "open" neighbor 
  %               with a distance of cell->distance - 1.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void callibrate( MazeCell * cell ) final {
    if( cell == nullptr ) return;
    int minDistance = INT_MAX;

    for( MazeCell * openNeighbor : cell->getNeighborList() ) {
      /* validate cell's need for callibration */
      if( openNeighbor->distance == cell->distance - 1 ) return;
      if( openNeighbor->distance < minDistance ) {
        minDistance = openNeighbor->distance;
      }
    }

    /* update non target cell to a higher elevation */
    if( cell->distance != 0 ) cell->distance = minDistance + 1;

    for( MazeCell * globalNeighbor : maze.getAdjacentCellList( cell ) ) {
      /* callibrate all global neighbors except for the target cells */
      if( globalNeighbor->distance == 0 ) continue;
      callibrate( globalNeighbor );
    }
  }

  /*****************************************************************************
  % Routine Name: retreat
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Continue exploring maze by returning to the starting position.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void retreat() final {
    MazeCell * newTargetCell = maze.at( start_position );
    start_position = std::make_pair( row, column );
    /* updates all cell distances with respect to new taget cell */
    updateMazeCellDistances( newTargetCell );
    /* current location pushed and ready */
    explore_stack.push_back( maze.at(row, column) );
    num_of_runs++;
  }

  /*****************************************************************************
  % Routine Name: trackSteps
  % File:         Mouse.hpp
  % Parameters:   Nothing.
  % Description:  Tracks the mouse's next maze traversal from starting point to 
  %               target point.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void trackSteps() final {
    mousePath.clear();
    updateMousePath( maze.at(start_position), maze.at(row, column) );
  }

  /*****************************************************************************
  % Routine Name: updateMousePath
  % File:         Mouse.hpp
  % Parameters:   start - starting cell of path.
  %               end   - terminating cell of path.
  % Description:  Appends path traversal to mousePath linked list. 
  % Return:       Nothing.
  *****************************************************************************/
  virtual void updateMousePath( MazeCell * start, MazeCell * end ) final {
    if( start == nullptr || end == nullptr ) return;
    mousePath.push_back( start );

    /* current node is at destination */
    if( *start == *end ) return;

    /* move to the next least expensive cell */
    for( MazeCell * neighbor : start->getNeighborList() ) {
      /* if mouse did not visit neighbor do not consider it */
      if( this->hasVisited( neighbor ) == false ) continue;
      /* otherwise append mouse path from neighbor to end */
      if( neighbor->distance == start->distance - 1 ) {
        updateMousePath( neighbor, end );
	return;
      }
    }
  }

  /*****************************************************************************
  % Routine Name: clearMemory
  % File:         Mouse.hpp
  % Parameters:   None.
  % Description:  Erases all memory about the maze configuration.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void clearMemory() final {
    /* erase all walls mouse created in maze */
    maze.clearWalls();
    /* erase memeory of expoloring maze */
    explore_stack.clear();
    mousePath.clear();
    previousPath.clear();
    num_of_runs = 0;
    done = false;

    for( MazeCell * cell : maze ) {
      MazeCell * center = getClosestCenter( cell );
      /* manhattan distance of cell to center */
      int dr = abs(center->row - cell->row);
      int dc = abs(center->column - cell->column);
      cell->setDistance( dr + dc );
      cell->setVisited( false );
      setVisited( cell, false );
    }
  }

  /*****************************************************************************
  % Routine Name: moveTo
  % File:         Mouse.hpp
  % Parameters:   coordinate - row, column pair.
  % Description:  Moves mouse to the given coordinate.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void moveTo( std::pair<int, int> & coordinate ) final {
    moveTo( maze.at(coordinate) );
  }

  /*****************************************************************************
  % Routine Name: moveTo
  % File:         Mouse.hpp
  % Parameters:   row - row to move to.
  %               column - column to move to.
  % Description:  Moves mouse to (row, column).
  % Return:       Nothing.
  *****************************************************************************/
  virtual void moveTo( int row, int column ) final {
    moveTo( maze.at(row, column) );
  }
  
  /*****************************************************************************
  % Routine Name: setVisited
  % File:         Mouse.hpp
  % Parameters:   cell - cell of interest.
  %               truthValue - visited value to set for cell.
  % Description:  Sets cell to the given truthValue.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void setVisited( MazeCell * cell, bool truthValue ) final {
    if( cell == nullptr ) return;
    visited[ cell->row ][ cell->column ] = truthValue;
  }

  /*****************************************************************************
  % Routine Name: setMousePosition
  % File:         Mouse.hpp
  % Parameters:   cell - cell to set the mouse to.
  % Description:  Updates the row and column of mouse to the given cell.
  % Return:       Nothing.
  *****************************************************************************/
  virtual void setMousePosition( MazeCell * cell ) final {
    // also check range
    if( cell == nullptr ) {
      #ifndef ARDUINO
        std::cerr << "Mouse.h:setMousePosition parameter invalid - defaulting "
	          << "to (" << origin.first << ", " << origin.second << ")" 
	          << std::endl;
      #endif
      cell = maze.at( origin );
    }
    row = cell->row;
    column = cell->column;
  }

  /*****************************************************************************
  % Routine Name: setMousePosition
  % File:         Mouse.hpp
  % Parameters:   coordinate - (row, column) pair.
  % Description:  Updates the mouse position to the given coordinate.
  % Return:       Nothing. 
  *****************************************************************************/
  virtual void setMousePosition( std::pair<int, int> & coordinate ) final {
    setMousePosition( maze.at(coordinate) );
  }

  /*****************************************************************************
  % Routine Name: setMousePosition
  % File:         Mouse.hpp
  % Parameters:   row - row in maze.
  %               column - column in maze.
  % Description:  Updates the mouse position to the given row, column.
  % Return:       Nothing. 
  *****************************************************************************/
  virtual void setMousePosition( int row, int column ) final {
    setMousePosition( maze.at(row, column) );
  }

  /*****************************************************************************
  % Routine Name: hasVisited
  % File:         Mouse.hpp
  % Parameters:   cell - cell of interest.
  % Description:  Evaluates if the given cell has been visited by the mouse.
  % Return:       true if the mouse visited the cell, false otherwise.
  *****************************************************************************/
  virtual bool hasVisited( MazeCell * cell ) final {
    if( cell == nullptr ) return false;
    return visited[ cell->row ][ cell->column ];
  }

  /*****************************************************************************
  % Routine Name: isCompletePath
  % File:         Mouse.hpp
  % Parameters:   path - path of interest.
  % Description:  Evaluates if the given path contains both the start and end 
  %               cell.
  % Return:       True if the path is complete, false otherwise.
  *****************************************************************************/
  virtual bool isCompletePath( std::vector<MazeCell *> & path ) final {
    if( path.size() == 0 ) {
      /* invlaid argument */
      #ifndef ARDUINO
        std::cerr << "Mouse.h:isCompletePath parameter is invalid: path size is"
	          << " 0" << std::endl;
      #endif
      return false;
    }

    MazeCell * mouse_cell = maze.at( row, column );
    MazeCell * start_cell = maze.at( start_position );

    bool path_contains_start = *path.front() == *start_cell 
                             || *path.back() == *start_cell;

    bool path_contains_mouse = *path.front() == *mouse_cell 
                             || *path.back() == *mouse_cell;

    if( path_contains_start && path_contains_mouse ) { 
      /* path contains both end points */
      return true;
    }
    return false;
  }

  /*****************************************************************************
  % Routine Name: getClosestCenter
  % File:         Mouse.hpp
  % Parameters:   cell - current cell.
  % Description:  Finds the center most cell with respect to the given cell.
  % Return:       The closest centered cell.
  *****************************************************************************/
  virtual MazeCell * getClosestCenter( MazeCell * cell ) final {
    const int EVEN = 2;

    if( maze.height != maze.width || cell == nullptr ) {
      /* non-square maze environment - no center or cell defined */
      return maze.at( 0, 0 );
    }

    const int maze_dimension = maze.height;
    int center_row = maze_dimension / EVEN;
    int center_column = maze_dimension / EVEN;

    /* singular solution cell */
    if( maze_dimension % EVEN == 1 ) {
      return maze.at( center_row, center_column );
    }
    
    /* quad-cell solution */
    if( cell->row < maze_dimension / EVEN ) {
      center_row = maze_dimension / EVEN - 1;
    }
    if( cell->column < maze_dimension / EVEN ) {
      center_column = maze_dimension / EVEN - 1;
    }
    return maze.at( center_row, center_column );
  }
};

#endif /* MOUSE_HPP */
