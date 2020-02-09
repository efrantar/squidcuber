/**
 * Implementation of matching the extracted BGR values to cube colors.
 */

#ifndef __MATCH__
#define __MATCH__

#include <string>

const std::string TBLFILE = "scan.tbl";

const int N_FACELETS = 54;

bool init_match();
// `n_attempts` is the maximum number of color options we explore per facelet; 3 is probably optimal here
std::string match_colors(const int bgrs[N_FACELETS][3], int n_attempts = 3);

#endif
