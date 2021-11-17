//
// Created by Dmitry on 13.11.2021.
//

#include "CSVPWM_NPC.h"
#include <cmath>

static const float PI = 3.1415926535897;
static const float PI_DIV_3 = PI/3.0;
static const float SQRT_3 = std::sqrt(3.0);

uint8_t CSVPWM_NPC::sector_identification(float _th) {
    if (_th < 0.0) _th += 2.0*PI;
    return m_sector = uint8_t(_th/PI_DIV_3);
}

std::pair<float, float> CSVPWM_NPC::m1m2_est(std::pair<float, float> _vector){
    if (_vector.second < 0) _vector.second += 2*PI;

    float _phiInSec = _vector.second - PI_DIV_3 * m_sector;

    float _cos = std::cos(_phiInSec);
    float _sin = std::sin(_phiInSec);

    m_m1m2.first = _vector.first * (_cos - 1.0/SQRT_3*_sin);
    m_m1m2.second = _vector.first * 2.0/SQRT_3*_sin;

    return m_m1m2;
}

uint8_t CSVPWM_NPC::subsector_identification() {
    if (m_m1m2.first + m_m1m2.second < 0.5){
        return m_subsector = 3;
    }
    if (m_m1m2.first >= 0.5)  return m_subsector = 0;
    if (m_m1m2.second >= 0.5)  return m_subsector = 2;
    return m_subsector = 1;
}

void CSVPWM_NPC::time_realization(eSupplyCapacitor_t _cap){
    switch (m_sector){
      case 0 :
        switch (m_subsector){
          case 0 : ccr<0, 0>(_cap); break;
          case 1 : ccr<0, 1>(_cap); break;
          case 2 : ccr<0, 2>(_cap); break;
          case 3 : ccr<0, 3>(_cap); break;
        }
        break;
      case 1 :
        switch (m_subsector){
          case 0 : ccr<1, 0>(_cap); break;
          case 1 : ccr<1, 1>(_cap); break;
          case 2 : ccr<1, 2>(_cap); break;
          case 3 : ccr<1, 3>(_cap); break;
        }
        break;
      case 2 :
        switch (m_subsector){
          case 0 : ccr<2, 0>(_cap); break;
          case 1 : ccr<2, 1>(_cap); break;
          case 2 : ccr<2, 2>(_cap); break;
          case 3 : ccr<2, 3>(_cap); break;
        }
        break;
      case 3 :
        switch (m_subsector){
          case 0 : ccr<3, 0>(_cap); break;
          case 1 : ccr<3, 1>(_cap); break;
          case 2 : ccr<3, 2>(_cap); break;
          case 3 : ccr<3, 3>(_cap); break;
        }
        break;
      case 4 :
        switch (m_subsector){
          case 0 : ccr<4, 0>(_cap); break;
          case 1 : ccr<4, 1>(_cap); break;
          case 2 : ccr<4, 2>(_cap); break;
          case 3 : ccr<4, 3>(_cap); break;
        }
        break;
      case 5 :
        switch (m_subsector){
          case 0 : ccr<5, 0>(_cap); break;
          case 1 : ccr<5, 1>(_cap); break;
          case 2 : ccr<5, 2>(_cap); break;
          case 3 : ccr<5, 3>(_cap); break;
        }
        break;
    }
}


template<>
void CSVPWM_NPC::ccr<0,0>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 1 - 2*m_m1m2.second;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 1;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first - 1;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<0,1>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 1 - 2*m_m1m2.second;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 2*m_m1m2.first;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1; // add 2*
}

template<>
void CSVPWM_NPC::ccr<0,2>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 2*(1 - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 0;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 2*m_m1m2.first;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<0,3>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
        m_ccr[4] = 1 - 2*m_m1m2.second; // add 2*
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[1] = 1 - 2*m_m1m2.second;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

//----- Sector 2 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<1,0>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first); // fixed first
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 0;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 2*m_m1m2.second;
    m_ccr[1] = 0;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<1,1>(eSupplyCapacitor_t _cap){
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.first; // fixed first
        m_ccr[4] = 0;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 2*m_m1m2.second;
    m_ccr[1] = 0;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<1,2>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.first;
        m_ccr[4] = 0;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 0;
    m_ccr[2] = 1;
    m_ccr[3] = 2*m_m1m2.second - 1;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<1,3>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.first;
        m_ccr[4] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
        m_ccr[5] = 1;
        return;
    }
    m_ccr[0] = 1 - 2*m_m1m2.first;
    m_ccr[1] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}


//----- Sector 3 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<2,0>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 1;
        m_ccr[4] = 0;
        m_ccr[5] = 1 - 2*m_m1m2.second;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 0;
    m_ccr[2] = 1;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second -1;
    m_ccr[4] = 0;
    m_ccr[5] = 2*m_m1m2.first - 1;
}

template<>
void CSVPWM_NPC::ccr<2,1>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 1;
        m_ccr[3] = 1;
        m_ccr[4] = 0;
        m_ccr[5] = 1 - 2*m_m1m2.second;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 0;
    m_ccr[2] = 2*m_m1m2.first;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second - 1; // add -1
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<2,2>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[2] = 2*(1 - m_m1m2.second);
        m_ccr[3] = 1;
        m_ccr[4] = 0;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 0;
    m_ccr[2] = 2*m_m1m2.first;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<2,3>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1;
        m_ccr[4] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
        m_ccr[5] = 1 - 2*m_m1m2.second;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[2] = 1 - 2*m_m1m2.second;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

//----- Sector 4 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<3,0>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 2*(1 - m_m1m2.first);
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 1;
        m_ccr[4] = 0;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 2*m_m1m2.second;
    m_ccr[2] = 0;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second - 1; // add -1
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<3,1>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 1;
        m_ccr[4] = (1 - 2*m_m1m2.first);
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 2*m_m1m2.second;
    m_ccr[2] = 0;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<3,2>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 1;
        m_ccr[4] = 1 - 2*m_m1m2.first;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 1;
    m_ccr[2] = 0;
    m_ccr[3] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[4] = 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<3,3>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1;
        m_ccr[4] = 1 - 2*m_m1m2.first;
        m_ccr[5] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;   //?????
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 1 - 2*m_m1m2.first;
    m_ccr[2] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

//----- Sector 5 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<4,0>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 1 - 2*m_m1m2.second;
        m_ccr[4] = 1;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 1;
    m_ccr[1] = 1;
    m_ccr[2] = 0;
    m_ccr[3] = 2*m_m1m2.first - 1;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<4,1>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 1 - 2*m_m1m2.second;
        m_ccr[4] = 1;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 2*m_m1m2.first;
    m_ccr[1] = 1;
    m_ccr[2] = 0;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<4,2>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[3] = 0;
        m_ccr[4] = 1;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 2*m_m1m2.first;
    m_ccr[1] = 1;
    m_ccr[2] = 0;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<4,3>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.second;
        m_ccr[4] = 1;
        m_ccr[5] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
        return;
    }
    m_ccr[0] = 1 - 2*m_m1m2.second;
    m_ccr[1] = 1;
    m_ccr[2] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

//----- Sector 6 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<5,0>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 2*(1 - m_m1m2.first);
        m_ccr[3] = 0;
        m_ccr[4] = 1;
        m_ccr[5] = 0;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 1;
    m_ccr[2] = 2*m_m1m2.second;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<5,1>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 1;
        m_ccr[5] = 1 - 2*m_m1m2.first;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 1;
    m_ccr[2] = 2*m_m1m2.second;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 0;
}

template<>
void CSVPWM_NPC::ccr<5,2>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 2*(1 - m_m1m2.first - m_m1m2.second);
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 0;
        m_ccr[4] = 1;
        m_ccr[5] = 1 - 2*m_m1m2.first;
        return;
    }
    m_ccr[0] = 0;
    m_ccr[1] = 1;
    m_ccr[2] = 1;
    m_ccr[3] = 0;
    m_ccr[4] = 2*m_m1m2.first + 2*m_m1m2.second - 1;
    m_ccr[5] = 2*m_m1m2.second - 1;
}

template<>
void CSVPWM_NPC::ccr<5,3>(eSupplyCapacitor_t _cap){
    
    if (_cap == eSupplyCapacitor_t::LO){
        m_ccr[0] = 1;
        m_ccr[1] = 1;
        m_ccr[2] = 1;
        m_ccr[3] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
        m_ccr[4] = 1;
        m_ccr[5] = 1 - 2*m_m1m2.first;
        return;
    }
    m_ccr[0] = 1 - 2*m_m1m2.first - 2*m_m1m2.second;
    m_ccr[1] = 1;
    m_ccr[2] = 1 - 2*m_m1m2.first;
    m_ccr[3] = 0;
    m_ccr[4] = 0;
    m_ccr[5] = 0;
}

