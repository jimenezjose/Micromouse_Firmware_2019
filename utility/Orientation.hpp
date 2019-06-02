/*******************************************************************************
                                                    Jose Jorge Jimenez-Olivas
                                                    Brandon Cramer
                                                    Chris Robles
                                                    Srinivas Venkatraman

                 University of California, San Diego
                      IEEE Micromouse Team 2019

File Name:       Orientation.hpp 
Description:     Orientational logic functions for navigation.
*******************************************************************************/
#ifndef ORIENTATION_HPP
#define ORIENTATION_HPP

/* Global enum compass */
enum Compass { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Orientation {
public:
  enum Compass orientation; 
 
  /*****************************************************************************
  % Constructor: Orientation 
  % File:        Orientation.hpp
  % Parameters:  orientation - enum compass direction of mouse.
  % Description: Creates an orientation object with relative utility methods.
  *****************************************************************************/
  Orientation( enum Compass orientation ) : orientation( orientation ) {}
 
 /*****************************************************************************
  % Routine Name: relativeRight
  % File:         Orientation.hpp 
  % Parameters:   None.
  % Description:  Gets the right compass position of the current orientation.
  % Return:       An enum compass direction that is relative right to current.
  *****************************************************************************/
  enum Compass relativeRight() {
    return directions[ (orientation + 1) % size() ];
  }

 /*****************************************************************************
  % Routine Name: relativeLeft
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  Gets the left compass position of the current orientation.
  % Return:       Enum compass direction that points relative left.
  *****************************************************************************/
  enum Compass relativeLeft() {
    return directions[ (orientation + (size() - 1)) % size() ];
  }

 /*****************************************************************************
  % Routine Name: relativeBack
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  Gets the compass direction of what is realtively back.
  % Return:       Enum compass of the realative back direction.
  *****************************************************************************/
  enum Compass relativeBack() {
    return directions[ (orientation + (size() / 2)) % size() ];
  }

 /*****************************************************************************
  % Routine Name: operator ++
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  pre-increment operator to change the current orientation to 
  %               relative right.
  % Return:       A reference to the calling object.
  *****************************************************************************/
  Orientation & operator++() {
    orientation = relativeRight();
    return *this;
  }

 /*****************************************************************************
  % Routine Name: operator ++
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  post-increment operator to change the current orientation to 
  %               relative right.
  % Return:       Orientation object before being incremented.
  *****************************************************************************/
  Orientation operator++(int) {
    Orientation before = *this;
    ++(*this);
    return before;
  }

 /*****************************************************************************
  % Routine Name: ordinal
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  Gets the index of the orientation with respect to the compass.
  % Return:       Integer index.
  *****************************************************************************/
  int ordinal() {
    return orientation;
  }

 /*****************************************************************************
  % Routine Name: size
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  Gets the number of directions in compass.
  % Return:       The number of directions in the compass.
  *****************************************************************************/
  int size() {
    return sizeof( directions ) / sizeof( Compass );
  }

 /*****************************************************************************
  % Routine Name: operator (const char *)
  % File:         Orientation.hpp
  % Parameters:   None
  % Description:  Casts the orientation object to a const char *.
  % Return:       String representatino of the orientation.
  *****************************************************************************/
  operator const char * () {
    switch( orientation ) {
      case NORTH:
        orientation_str = "NORTH";
        break;
      case EAST:
        orientation_str = "EAST";
        break;
      case SOUTH:
        orientation_str = "SOUTH";
        break;
      case WEST:
        orientation_str = "WEST";
        break;
    }
    return orientation_str.c_str();
  }

 /*****************************************************************************
  % Routine Name: operator (enum Compass)
  % File:         Orientation.hpp
  % Parameters:   None.
  % Description:  Casts the orientation object to a compass enum.
  % Return:       The current orientation of the object.
  *****************************************************************************/
  operator enum Compass () {
    return orientation;
  }

private:
  enum Compass directions[ 4 ] = { NORTH, EAST, SOUTH, WEST };
  std::string orientation_str;
};

#endif /* ORIENTATION_HPP */
