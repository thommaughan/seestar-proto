/*******************************************************************************
SerIn - An Arduino library to parse commands and arguments received over
a serial port.   Derived from SerialCommand by Steven Cogswell.


See SerIn.h for version history.

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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SerCmd.h"
#include <string.h>


// Constructor makes sure some things are set.
SerCmd::SerCmd()
{
	//default delim is :
	strncpy(delim," .,:=",MAXDELIMETER);  // strtok_r needs a null-terminated string
	term = '\r';           // return character, default terminator for commands
	numCommand = 0;        // Number of callback handlers installed
	clearRxBuf();
}


//
// Initialize the command buffer being processed to all null characters
//
void SerCmd::clearRxBuf()
{
	for (int i=0; i<SIZE_RXBUF; i++)
	{
		rxbuf[i] = 0x00;
	}
	bufPos = 0;
}

// Retrieve the next token ("word" or "argument") from the Command buffer.
// returns a NULL if no more tokens exist.
char *SerCmd::next()
{
	char *nextToken;
	nextToken = strtok_r(NULL, delim, &last);
	return nextToken;
}

// Retrieve the next token ("word" or "argument") from the Command buffer.
// returns a NULL if no more tokens exist.
char *SerCmd::next(char sep)
{
	char *nextToken;
	char tmpDelim[2];

	tmpDelim[0] = sep;
	tmpDelim[1] = 0x00;
	nextToken = strtok_r(NULL, tmpDelim, &last);
	return nextToken;
}

// get private numCommand
int SerCmd::getNumCmd()
{
	return numCommand;
}

// get a pointer to the command string for printing the command menu
char *SerCmd::getCmd(int cmdIndx)
{
	//Serial.println(CommandList[cmdIndx].command);
	return CommandList[cmdIndx].command;
}

// This checks the Serial stream for characters, and assembles them into a buffer.
// When the terminator character (default '\r') is seen, it starts parsing the
// buffer for a prefix command, and calls handlers setup by addCommand() member
void SerCmd::readSerial()
{
	while (Serial.available() > 0)
	{
		int i;
		boolean matched;
		inChar=Serial.read();   // Read single available character, there may be more waiting

		#ifdef SERIALCOMMANDDEBUG
		Serial.print(inChar);   // Echo back to serial stream
		#endif
		if (inChar==term)
		{     // Check for the terminator (default '\r') meaning end of command
			#ifdef SERIALCOMMANDDEBUG
			Serial.print("Received: ");
			Serial.println(rxbuf);
		    #endif
		    rxCmd = 0;
			if(bufPos < 2)
				rxCmd = rxbuf[0];
			bufPos=0;           // Reset to start of buffer
			token = strtok_r(rxbuf,delim,&last);   // Search for command at start of buffer
			if (token == NULL)
				return;

			matched=false;
			for (i=0; i<numCommand; i++)
			{
				#ifdef SERIALCOMMANDDEBUG
				Serial.print("Comparing [");
				Serial.print(token);
				Serial.print("] to [");
				Serial.print(CommandList[i].command);
				Serial.println("]");
				#endif

				// Compare the found command against the list of known commands for a match
				if (strncmp(strupr(token),CommandList[i].command,MAXSIZE_SERIALCOMMAND) == 0)
				//if (strnicmp(token,CommandList[i].command,MAXSIZE_SERIALCOMMAND) == 0)  // no strnicmp exists in AVR
				{
					#ifdef SERIALCOMMANDDEBUG
					Serial.print("Matched Command: ");
					Serial.println(token);
					#endif

					// Execute the stored handler function for the command
					(*CommandList[i].function)();
					clearRxBuf();
					matched=true;
					break;
				}

			}
			if (matched==false)
			{
				(*defaultHandler)();
				clearRxBuf();
			}

		}  // if inchar is terminaor

		if (isprint(inChar))   // Only printable characters into the buffer
		{
			rxbuf[bufPos++] = inChar;   // Put character into buffer
			rxbuf[bufPos] = '\0';  // Null terminate
			if (bufPos > SIZE_RXBUF-1) bufPos=0; // wrap buffer around if full
		}
	}
}

// Adds a "command" and a handler function to the list of available commands.
// This is used for matching a found token in the buffer, and gives the pointer
// to the handler function to deal with it.
void SerCmd::addCommand(const char *command, void (*function)())
{
	if (numCommand < MAXSERIALCOMMANDS) {
		#ifdef SERIALCOMMANDDEBUG
		Serial.print(numCommand);
		Serial.print("-");
		Serial.print("Adding command for ");
		Serial.println(command);
		#endif

		strncpy(CommandList[numCommand].command,command,MAXSIZE_SERIALCOMMAND);
		CommandList[numCommand].function = function;
		numCommand++;
	} else {
		// In this case, you tried to push more commands into the buffer than it is compiled to hold.
		// Not much we can do since there is no real visible error assertion, we just ignore adding
		// the command
		#ifdef SERIALCOMMANDDEBUG
		Serial.println("Too many handlers - recompile changing MAXSERIALCOMMANDS");
		#endif
	}
}

// This sets up a handler to be called in the event that the receveived command string
// isn't in the list of things with handlers.
void SerCmd::addDefaultHandler(void (*function)())
{
	defaultHandler = function;
}

char SerCmd::getCharCmd(void)    // returns 0 (no command) or ascii single letter command
{
	char tmp;

	tmp = rxCmd;
	rxCmd = 0;
    return(tmp);
}