/*

Module:	header_test.ino

Function:
	Test header file for MCCI_Catena_LTR329 library

Copyright and License:
	See accompanying LICENSE file for copyright and license information.

Author:
	Pranau, MCCI Corporation    June 2022

*/

#include <MCCI_Catena_LTR329.h>

static_assert(
	McciCatenaLtr329::kVersion > McciCatenaLtr329::makeVersion(0,0,0),
	"version must be > 0.0.0"
	);

void setup()
	{
	}

void loop()
	{
	}