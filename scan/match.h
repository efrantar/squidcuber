#ifndef __MATCH__
#define __MATCH__

#include <string>

const std::string TBLFILE = "scan.tbl";

bool init();
std::string match_colors(int bgrs[]);

#endif
