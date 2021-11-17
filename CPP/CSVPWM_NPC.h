//
// Created by Dmitry on 13.11.2021.
//

#ifndef SWPMW_NPC_CSVPWM_NPC_H
#define SWPMW_NPC_CSVPWM_NPC_H

#include <cstdint>
#include <utility>

class CSVPWM_NPC {
  public:
    enum class eSupplyCapacitor_t {LO = 0, HI};
  private:
    uint8_t m_sector = 0;
    uint8_t m_subsector = 0;
    std::pair<float, float> m_m1m2;

    float m_ccr[6];

  private:
    uint8_t sector_identification(float _th);
    std::pair<float, float> m1m2_est(std::pair<float,float> _vector);
    uint8_t subsector_identification();
    void time_realization(eSupplyCapacitor_t);



  public:
    const float* ccr_est(std::pair<float, float> _vector, eSupplyCapacitor_t _cap){
        sector_identification(_vector.second);
        m1m2_est(_vector);
        subsector_identification();
        time_realization(_cap);
        for (int i = 0; i < 6; ++i) m_ccr[i] = 1 - m_ccr[i];
        return m_ccr;
    }
    uint8_t sector_get(){return m_sector;}
    uint8_t subsector_get(){return m_subsector;}

  private:
    template <uint8_t sectorNo, uint8_t _subsectorNo>
        void ccr(eSupplyCapacitor_t _cap);
        

};



//----- Sector 1 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<0,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<0,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<0,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<0,3>(eSupplyCapacitor_t _cap);

//----- Sector 2 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<1,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<1,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<1,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<1,3>(eSupplyCapacitor_t _cap);


//----- Sector 3 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<2,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<2,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<2,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<2,3>(eSupplyCapacitor_t _cap);

//----- Sector 4 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<3,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<3,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<3,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<3,3>(eSupplyCapacitor_t _cap);

//----- Sector 5 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<4,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<4,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<4,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<4,3>(eSupplyCapacitor_t _cap);

//----- Sector 6 ---------------------------------------------------------------

template<>
void CSVPWM_NPC::ccr<5,0>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<5,1>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<5,2>(eSupplyCapacitor_t _cap);

template<>
void CSVPWM_NPC::ccr<5,3>(eSupplyCapacitor_t _cap);
#endif //SWPMW_NPC_CSVPWM_NPC_H
