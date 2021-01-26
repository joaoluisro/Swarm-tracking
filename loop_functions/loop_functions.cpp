#include "loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <footbot_tracking/footbot_tracking.h>

// inicializa variáveis globais de : arena + informações do swarm

CTrackingLoopFunctions::CTrackingLoopFunctions() :
   m_cForagingArenaSideX(-0.9f, 1.7f),
   m_cForagingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) {
}

// Inicializa o experimento

void CTrackingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      m_pcFloor = &GetSpace().GetFloorEntity();
      // numero de alvos
      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      // gerador de numeros aleatórios
      m_pcRNG = CRandom::CreateRNG("argos");
      // distribuição dos alvos
      for(UInt32 i = 0; i < unFoodItems; ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
      GetNodeAttribute(tForaging, "output", m_strOutput);
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# iteração\tprocurando\tdescanso\talvos_encontrados\tenergia" << std::endl;
      GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}


void CTrackingLoopFunctions::Reset() {
   m_unCollectedFood = 0;
   m_nEnergy = 0;
   m_cOutput.close();
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
   }
}


void CTrackingLoopFunctions::Destroy() {
   m_cOutput.close();
}


CColor CTrackingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}


void CTrackingLoopFunctions::PreStep() {
   // função que dita o funcionamento de encontro ao alvo
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   // robô encontrou o alvo?

   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end();it++) {
      // acessa entidade foot-bot e controladores
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      FootBotTrack& cController = dynamic_cast<FootBotTrack&>(cFootBot.GetControllableEntity().GetController());
      // conta quantos robôs estão em quantos estados
      if(! cController.IsResting()) unWalkingFBs++;
      else unRestingFBs++;

      // cPos -> posição do robô no plano 2D
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      // posição do alvo
      FootBotTrack::Alvo& Alvo = cController.GetInfoAlvo();

      if(Alvo.AlvoSpotted){
        std::cout << cPos.GetX() << cPos.GetY();
      }

      // alvo não foi encontrado
      if(!Alvo.AlvoSpotted) {

        // se x > -1.0 => robô esta fora da zona de descanso
        if(cPos.GetX() > -1.0f) {
           bool bDone = false;
           for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
              if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                 m_cFoodPos[i].Set(100.0f, 100.f);
                 Alvo.AlvoSpotted = true;
                 Alvo.AlvoID = i;
                 m_pcFloor->SetChanged();
                 bDone = true;
              }
           }
        }
      }
   }
   m_nEnergy -= unWalkingFBs * m_unEnergyPerWalkingRobot;
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"
             << m_nEnergy << std::endl;
}

REGISTER_LOOP_FUNCTIONS(CTrackingLoopFunctions, "loop_functions")
