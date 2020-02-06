/**
 * Implementation of matching the extracted BGR values to cube colors.
 */

#ifndef __MATCH__
#define __MATCH__

#include <string>

const std::string TBLFILE = "scan.tbl";

const int N_FACELETS = 54;

bool init_match();
std::string match_colors(const int bgrs[N_FACELETS][3]);

#endif
