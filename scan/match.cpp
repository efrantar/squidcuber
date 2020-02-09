/**
 * This file implements the actual color matching. Colors are assigned in order of confidence as given by the
 * precomputed scan-table (from a KNN learned on successful scans) while taking into account all cube constraints.
 * Properly (and efficiently) implementing full constraint propagation is quite tricky, yet very powerful. Together
 * with reliable confidence scores this makes for a very robust scanning method that is consistently able to handle
 * strong reflections and varying lighting.
 */

#include "match.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <vector>


/* Since scanner and solver should be independent programs, we need to redefine several important constants here. */

namespace color {
  const int COUNT = 6;

  const int U = 0;
  const int R = 1;
  const int F = 2;
  const int D = 3;
  const int L = 4;
  const int B = 5;

  const int CORNERS[][3] = {
    {U, R, F}, {U, F, L}, {U, L, B}, {U, B, R},
    {D, F, R}, {D, L, F}, {D, B, L}, {D, R, B}
  };

  const int EDGES[][2] = {
    {U, R}, {U, F}, {U, L}, {U, B},
    {D, R}, {D, F}, {D, L}, {D, B},
    {F, R}, {F, L}, {B, L}, {B, R}
  };

  const char CHARS[] = {'U', 'R', 'F', 'D', 'L', 'B'};
}

namespace cubie {

  const int N_EDGES = 12;
  const int N_CORNERS = 8;

  const int URF = 0;
  const int UFL = 1;
  const int ULB = 2;
  const int UBR = 3;
  const int DFR = 4;
  const int DLF = 5;
  const int DBL = 6;
  const int DRB = 7;

  const int UR = 0;
  const int UF = 1;
  const int UL = 2;
  const int UB = 3;
  const int DR = 4;
  const int DF = 5;
  const int DL = 6;
  const int DB = 7;
  const int FR = 8;
  const int FL = 9;
  const int BL = 10;
  const int BR = 11;

  // Map a facelet to the cubie it's on
  const int FROM_FACELET[] = {
    ULB, UB, UBR, UL, -1, UR, UFL, UF, URF,
    URF, UR, UBR, FR, -1, BR, DFR, DR, DRB,
    UFL, UF, URF, FL, -1, FR, DLF, DF, DFR,
    DLF, DF, DFR, DL, -1, DR, DBL, DB, DRB,
    ULB, UL, UFL, BL, -1, FL, DBL, DL, DLF,
    UBR, UB, ULB, BR, -1, BL, DRB, DB, DBL
  };

}

// Map a facelet to its position on the corresponding cubie
const int FACELET_TO_POS[] = {
  0, 0, 0, 0, -1, 0, 0, 0, 0,
  1, 1, 2, 1, -1, 1, 2, 1, 1,
  1, 1, 2, 0, -1, 0, 2, 1, 1,
  0, 0, 0, 0, -1, 0, 0, 0, 0,
  1, 1, 2, 1, -1, 1, 2, 1, 1,
  1, 1, 2, 0, -1, 0, 2, 1, 1
};

using colset_t = uint8_t;


// Lookup the precomputed confidence values learned by KNN for every possible BGR-color
const int N_BGRS = 16777216;
uint16_t (*scantbl)[color::COUNT];

template <int n_cubies, int n_oris, const int cubiecols[n_cubies][n_oris]>
class Options {

  struct Opt { // tight memory to make copying faster
    uint8_t cols[n_oris];
    colset_t colset;
    uint8_t ori;
    uint8_t cubie;
  };

  Opt opts[n_cubies * n_oris];
  int rem;

  bool error;
  colset_t colset;
  int ori;
  int cubie;

  void update() {
    if (rem == 0) {
      error = true;
      return;
    }

    colset = opts[0].colset; // at this point there is at least one option left
    for (int i = 1; i < rem; i++)
      colset &= opts[i].colset;

    if (ori == -1) {
      int singleori = opts[0].ori;
      for (int i = 1; i < rem; i++) {
        if (opts[i].ori != singleori) {
          singleori = -1;
          break;
        }
      }
      if (singleori != -1)
        ori = singleori;
    }

    if (cubie == -1) {
      int singlecubie = opts[0].cubie;
      for (int i = 1; i < rem; i++) {
        if (opts[i].cubie != singlecubie) {
          singlecubie = -1;
          break;
        }
      }
      if (singlecubie != -1)
        cubie = singlecubie;
    }
  }

  public:

    void init() { // no constructor to make it a POD allowing super fast mem-copying
      int i = 0;
      for (int cubie = 0; cubie < n_cubies; cubie++) {
        for (int ori = 0; ori < n_oris; ori++) {
          opts[i].cubie = cubie;
          opts[i].ori = ori;
          for (int j = 0; j < n_oris; j++) {
            opts[i].cols[j] = cubiecols[cubie][(j + ori) % n_oris];
            opts[i].colset |= 1 << opts[i].cols[j];
          }
          i++;
        }
      }
      rem = n_cubies * n_oris;

      error = false;
      colset = 0;
      ori = - 1;
      cubie = -1;
    }

    bool get_error() {
      return error;
    }

    colset_t get_colset() {
      return colset;
    }

    int get_ori() {
      return ori;
    }

    int get_cubie() {
      return cubie;
    }

    void has_poscol(int pos, int col) {
      int rem1 = 0;
      for (int i = 0; i < rem; i++) {
        if (opts[i].cols[pos] == col)
          opts[rem1++] = opts[i];
      }
      if (rem1 != rem) {
        rem = rem1;
        update();
      } else
        rem = rem1;
    }

    void hasnot_col(int col) {
      int rem1 = 0;
      for (int i = 0; i < rem; i++) {
        if ((opts[i].colset & (1 << col)) == 0)
          opts[rem1++] = opts[i];
      }
      if (rem1 != rem) {
        rem = rem1;
        update();
      }
    }

    void has_ori(int ori) {
      int rem1 = 0;
      for (int i = 0; i < rem; i++) {
        if (opts[i].ori == ori)
          opts[rem1++] = opts[i];
      }
      if (rem1 != rem) {
        rem = rem1;
        update();
      }
    }

    void is_cubie(int cubie) {
      int rem1 = 0;
      for (int i = 0; i < rem; i++) {
        if (opts[i].cubie == cubie)
          opts[rem1++] = opts[i];
      }
      if (rem1 != rem) {
        rem = rem1;
        update();
      }
    }

    void isnot_cubie(int cubie) {
      int rem1 = 0;
      for (int i = 0; i < rem; i++) {
        if (opts[i].cubie != cubie)
          opts[rem1++] = opts[i];
      }
      if (rem1 != rem) {
        rem = rem1;
        update();
      }
    }

};

template <int n_cubies, int n_oris, const int cubiecols[n_cubies][n_oris]>
class CubieBuilder {

  int colcounts[color::COUNT];
  colset_t colsets[n_cubies];
  int oris[n_cubies];
  int perm[n_cubies];
  int par;

  Options<n_cubies, n_oris, cubiecols> opts[n_cubies];
  int invcnt;
  int orisum;
  int aperm;
  int aoris;

  bool assign_cubie(int i) {
    int cubie = opts[i].get_cubie();
    if (cubie == -1 || perm[i] != -1)
      return false;

    perm[i] = cubie;
    for (int j = 0; j < i; j++) {
      if (perm[j] != - 1 && perm[j] > cubie)
        invcnt++;
    }
    for (int j = i; j < n_cubies; j++) {
      if (perm[j] != -1 && perm[j] < cubie)
        invcnt++;
    }
    aperm++;
    if (aperm == n_cubies)
      par = invcnt & 1; // permutation fully determined -> compute parity

    // Every cubie exists exactly once -> eliminate from all other options
    for (int j = 0; j < n_cubies; j++) {
      if (j != i)
        opts[j].isnot_cubie(cubie);
    }

    return true;
  }

  bool assign_ori(int i) {
    int ori = opts[i].get_ori();
    if (ori == -1 || oris[i] != -1)
      return false;

    oris[i] = ori;
    orisum += ori;
    aoris++;
    return true;
  }

  public:

    void init() { // again, we want to keep this a POD
      std::fill(colcounts, colcounts + color::COUNT, 4);
      std::fill(colsets, colsets + n_cubies, 0);
      std::fill(oris, oris + n_cubies, -1);
      std::fill(perm, perm + n_cubies, -1);
      par = -1;

      for (auto& o : opts)
        o.init();

      invcnt = 0;
      orisum = 0;
      aperm = 0;
      aoris = 0;
    }

    int get_par() {
      return par;
    }

    void assign_col(int cubie, int pos, int col) {
      opts[cubie].has_poscol(pos, col);
    }

    void assign_par(int par) {
      this->par = par;
    }

    bool propagate() {
      bool change = true;
      while (change) {
        change = false;

        for (int cubie = 0; cubie < n_cubies; cubie++) {
          if (opts[cubie].get_error())
            return false;

          colset_t diff = opts[cubie].get_colset() ^colsets[cubie]; // latter will always be included in former
          colsets[cubie] |= diff;
          for (int col = 0; col < color::COUNT; col++) {
            if ((diff & (1 << col)) == 0)
              continue;
            colcounts[col]--;
            if (colcounts[col] == 0) { // all cubies of some color known
              for (int i = 0; i < n_cubies; i++) {
                // Some `colset` update might not have been registered yet
                if ((opts[i].get_colset() & (1 << col)) == 0) {
                  opts[i].hasnot_col(col);
                  change = true;
                }
              }
            }
          }

          change |= assign_ori(cubie);
          change |= assign_cubie(cubie);
        }

        // Figure out last ori by parity
        if (aoris == n_cubies - 1) {
          int lastori = (n_oris - orisum % n_oris) % n_oris;
          for (int i = 0; i < n_cubies; i++) {
            if (oris[i] == -1) {
              opts[i].has_ori(lastori);
              // Assign only in next iteration to not accidentally overrule contradictions
              break;
            }
          }
          change = true;
        }

        // Figure out position of last two cubies by parity
        if (par != -1 && aperm == n_cubies - 2) {
          int i1 = std::find(perm, perm + n_cubies, -1) - perm;
          int i2 = std::find(perm + i1 + 1, perm + n_cubies, -1) - perm;

          bool contained[n_cubies] = {};
          for (int c : perm) {
            if (c != -1)
              contained[c] = true;
          }
          int cubie1 = 0;
          while (contained[cubie1])
            cubie1++;
          int cubie2 = cubie1 + 1;
          while (contained[cubie2])
            cubie2++;

          int invcnt1 = 0;
          for (int i = 0; i < n_cubies; i++) {
            if (perm[i] == -1)
              continue;
            invcnt1 += i < i1 && perm[i] > cubie1;
            invcnt1 += i > i1 && perm[i] < cubie1;
            invcnt1 += i < i2 && perm[i] > cubie2;
            invcnt1 += i > i2 && perm[i] < cubie2;
          }
          if (((invcnt + invcnt1) & 1) != par)
            std::swap(i1, i2); // flip cubie positions to fix parity

          opts[i1].is_cubie(cubie1);
          opts[i2].is_cubie(cubie2);
          change = true;
        }
      }

      return true;
    }

};

using CornersBuilder = CubieBuilder<cubie::N_CORNERS, 3, color::CORNERS>;
using EdgesBuilder = CubieBuilder<cubie::N_EDGES, 2, color::EDGES>;

std::string match_colors(const int bgrs[N_FACELETS][3], int n_attempts) {
  int facecube[N_FACELETS];

  int conf[N_FACELETS][color::COUNT];
  for (int f = 0; f < N_FACELETS; f++) {
    for (int col = 0; col < color::COUNT; col++)
      conf[f][col] = scantbl[256 * (256 * bgrs[f][0] + bgrs[f][1]) + bgrs[f][2]][col];
  }

  std::priority_queue<std::tuple<int, int, int>> heap;
  for (int f = 0; f < N_FACELETS; f++) {
    if (f % 9 == 4) // centers are fixed
      facecube[f] = f / 9;
    else {
      int imax = std::max_element(conf[f], conf[f] + color::COUNT) - conf[f];
      heap.emplace(conf[f][imax], f, imax);
      conf[f][imax] = -1; // makes it easy to find the next largest index
    }
  }
  int attempts[N_FACELETS];
  std::fill(attempts, attempts + N_FACELETS, n_attempts);

  // Pointers to simply swap backups back in instead of having to copy them again
  auto* corners = new CornersBuilder();
  auto* edges = new EdgesBuilder();
  corners->init();
  edges->init();
  auto* corners1 = new CornersBuilder();
  auto* edges1 = new EdgesBuilder();

  while (!heap.empty()) {
    auto ass = heap.top();
    heap.pop();

    int f = std::get<1>(ass);
    int cubie = cubie::FROM_FACELET[f];
    int pos = FACELET_TO_POS[f];
    int col = std::get<2>(ass);

    bool succ;
    if ((f % 9) % 2 == 1) { // is on an edge
      memcpy(edges1, edges, sizeof(*edges));
      edges->assign_col(cubie, pos, col);
      if (!(succ = edges->propagate()))
        std::swap(edges1, edges);
      else if (edges->get_par() != -1 && corners->get_par() == -1) {
        corners->assign_par(edges->get_par());
        if (!(succ = edges->propagate())) {
          std::swap(edges1, edges);
          std::swap(corners1, corners);
        }
      }
    } else {
      memcpy(corners1, corners, sizeof(*corners));
      corners->assign_col(cubie, pos, col);
      if (!(succ = corners->propagate()))
        std::swap(corners1, corners);
      else if (corners->get_par() != -1 && edges->get_par() == -1) {
        memcpy(edges1, edges, sizeof (*edges));
        edges->assign_par(corners->get_par());
        if (!(succ = edges->propagate())) {
          std::swap(corners1, corners);
          std::swap(edges1, edges);
        }
      }
    }

    if (!succ) {
      auto tmp = std::max_element(conf[f], conf[f] + color::COUNT);
      if (*tmp == -1)
        return ""; // scan error
      int imax = tmp - conf[f];
      heap.emplace(conf[f][imax], f, imax);
      conf[f][imax] = -1;
      if (--attempts[f] < 0)
        return ""; // this is mostly to prevent too much constraint forcing on scan errors
    } else
      facecube[f] = col;
  }

  char s[N_FACELETS];
  for (int i = 0; i < N_FACELETS; i++)
    s[i] = color::CHARS[facecube[i]];
  return std::string(s, N_FACELETS);
}

bool init_match() {
  FILE *f = fopen(TBLFILE.c_str(), "rb");
  if (f == NULL)
    return false;
  scantbl = new uint16_t[N_BGRS][color::COUNT];
  bool succ = fread(scantbl, sizeof(uint16_t[N_BGRS][color::COUNT]), 1, f) == 1;
  fclose(f);
  return succ;
}

/*
int main() {
  if (!init_match()) {
    std::cout << "Error loading table." << std::endl;
    return 0;
  }

  int bgrs[][3] = {
    { 96, 149,  75},
    {117,  31,  10},
    {227, 203, 198},
    { 17, 221, 245},
    {  0, 114, 214},
    { 25, 155, 165},
    {180, 225, 236},
    { 92,  24,   5},
    { 20, 159, 174},
    {169, 147, 149},
    {139, 184, 130},
    { 70, 142, 248},
    {110, 137, 180},
    { 10, 199, 226},
    {254, 255, 251},
    {111, 142, 182},
    { 88, 115, 165},
    { 17,  35, 135},
    {111, 169, 250},
    {133, 142, 208},
    { 98, 129, 212},
    {162, 255, 254},
    { 80,  44,  22},
    {204, 212, 228},
    {104, 168,  99},
    { 93, 129,  84},
    {113,  83,  80},
    {136, 139, 139},
    {161, 159, 158},
    {174, 167, 164},
    { 91,  42,  26},
    {  4,  10,  71},
    {134, 140, 100},
    { 66, 134, 134},
    {126, 132,  93},
    {133, 143, 106},
    { 74, 162, 184},
    { 87, 112, 204},
    {120,  79,  63},
    {113,  88,  85},
    {152, 159, 162},
    { 67,  66, 116},
    { 90,  60,  56},
    {152, 171, 179},
    { 38,  40,  98},
    {160, 193,  97},
    { 58,  65, 119},
    { 91, 120, 192},
    { 52, 113, 232},
    { 93, 122,  41},
    { 87, 172, 177},
    { 91, 218, 218},
    { 79, 115, 202},
    {100, 100, 115}
  };

  auto tick = std::chrono::high_resolution_clock::now();
  std::cout << match_colors(bgrs) << std::endl;
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - tick
  ).count() / 1000. << "ms" << std::endl;

  return 0;
}
*/
