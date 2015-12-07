/*******************************************************************************
SerCmd is derived from SerialCommand (gnu 2.1) by Steven Cogswell, thanks Steven!

Differences:
Command strings are in PROGMEM, not precious SRAM.  Delimiter tokens are
programmable, and command case is ignored.


Version History:
15 Aug 2015 - Initial version derived from SerialCommand library.
26 Aug 2015 - Remove trailing white space from command, allow for white space in command


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
***********************************************************************************/
#ifndef SerCmd_h
#define SerCmd_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>


#define MAXSIZE_SERIALCOMMAND	 12
#define SIZE_RXBUF				 40		//SERIALCOMMANDBUFFER_SIZE 25			// was 16
#define MAXSERIALCOMMANDS		 20				// was 10
#define MAXDELIMETER              7     //2

#define SERIALCOMMANDDEBUG 1
#undef  SERIALCOMMANDDEBUG      // Comment this out to run the library in debug mode (verbose messages)

class SerCmd
{
	public:
		SerCmd();      // Constructor

		void clearRxBuf();   // Sets the command buffer to all '\0' (nulls)
		char *next();         // returns pointer to next token found in command buffer (for getting arguments to commands)
		char *next(char sep);
		void readSerial();    // Main entry point.
		void addCommand(const char *, void(*)());   // Add commands to processing dictionary
		//void addCommand(unsigned char, void(*)());   // Add commands to processing dictionary

		char *getCmd(int cmdIndx);		  // get command
		void addDefaultHandler(void (*function)());    // A handler to call when no valid command received.
		int	 getNumCmd();				// read private numCommand
		char getCharCmd();    // returns 0 (no command) or ascii single letter command
	private:
		char inChar;          // A character read from the serial stream
		char rxCmd;
		char rxbuf[SIZE_RXBUF];   // Buffer of stored characters while waiting for terminator character
		int  bufPos;                        // Current position in the buffer
		char delim[MAXDELIMETER];           // null-terminated list of character to be used as delimeters for tokenizing (default " ")
		char term;                          // Character that signals end of command (default '\r')
		char *token;                        // Returned token from the command buffer as returned by strtok_r
		char *last;                         // State variable used by strtok_r during processing
		typedef struct _callback
		{
			char command[MAXSIZE_SERIALCOMMAND];
			unsigned char tableIndx;
			void (*function)();
		} SerCmdCallback;            // Data structure to hold Command/Handler function key-value pairs
		int numCommand;
		SerCmdCallback CommandList[MAXSERIALCOMMANDS];   // Actual definition for command/handler array (tableIndx corresponds to command)
		void (*defaultHandler)();           // Pointer to the default handler function
};

#endif //SerCmd_h