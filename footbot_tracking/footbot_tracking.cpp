#include "footbot_tracking.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

FootBotTrack::Alvo::Alvo() :
   AlvoSpotted(false),
   AlvoID(0),
   TotalAlvos(0) {}

void FootBotTrack::Alvo::Reset() {
   AlvoSpotted = false;
   AlvoID = 0;
   TotalAlvos = 0;
}

// parametros de do algoritmo de difusão

FootBotTrack::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void FootBotTrack::SDiffusionParams::Init(TConfigurationNode& t_node) {
  CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
  GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
  GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
  GetNodeAttribute(t_node, "delta", Delta);
}

// parametros de movimento da roda

void FootBotTrack::SWheelTurningParams::Init(TConfigurationNode& t_node) {
  TurningMechanism = NO_TURN;
  CDegrees cAngle;
  GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
  HardTurnOnAngleThreshold = ToRadians(cAngle);
  GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
  SoftTurnOnAngleThreshold = ToRadians(cAngle);
  GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
  NoTurnAngleThreshold = ToRadians(cAngle);
  GetNodeAttribute(t_node, "max_speed", MaxSpeed);
}

FootBotTrack::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void FootBotTrack::SStateData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
    GetNodeAttribute(t_node, "initial_explore_to_rest_prob", InitialExploreToRestProb);
    GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", FoodRuleExploreToRestDeltaProb);
    GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", FoodRuleRestToExploreDeltaProb);
    GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", CollisionRuleExploreToRestDeltaProb);
    GetNodeAttribute(t_node, "social_rule_rest_to_explore_delta_prob", SocialRuleRestToExploreDeltaProb);
    GetNodeAttribute(t_node, "social_rule_explore_to_rest_delta_prob", SocialRuleExploreToRestDeltaProb);
    GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
    GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
    GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
}

void FootBotTrack::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = InitialRestToExploreProb;
   ExploreToRestProb = InitialExploreToRestProb;
   TimeExploringUnsuccessfully = 0;
   TimeRested = MinimumRestingTime;
   TimeSearchingForPlaceInNest = 0;
}


FootBotTrack::FootBotTrack() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL) {}

void FootBotTrack::Init(TConfigurationNode& t_node) {

  m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
  m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
  m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
  m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
  m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
  m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );

  m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));

  m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

  m_sStateData.Init(GetNode(t_node, "state"));


   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}


void FootBotTrack::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         FoundTarget();
         break;
      }
      default: {
         LOGERR << "erro: estado desconhecido" << std::endl;
      }
   }
}


void FootBotTrack::Reset() {
   m_sStateData.Reset();
   m_Alvo.Reset();
   m_pcLEDs->SetAllColors(CColor::RED);
   m_eLastExplorationResult = LAST_EXPLORATION_NONE;
   m_pcRABA->ClearData();
   m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
}


void FootBotTrack::UpdateState() {
   m_sStateData.InNest = false;
   // capta informações do sensor
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();

    // verifica se está na área inicial
   if(tGroundReads[2].Value > 0.25f &&
      tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f &&
      tGroundReads[3].Value < 0.75f) {
      m_sStateData.InNest = true;
   }
}

// função foraging que captura luz dos sensores

CVector2 FootBotTrack::CalculateVectorToLight() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

// vetor de difusão

CVector2 FootBotTrack::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

// principal função de movimento, recebe uma direção <x,y>
// faz com que o robõ se direcione a ela

void FootBotTrack::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   Real fHeadingLength = c_heading.Length();
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}


// Rest() faz com que o robõ "descanse" e carregue sua bateria (caso ele possua uma)

void FootBotTrack::Rest() {

   if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
      m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.RestToExploreProb) {
      m_pcLEDs->SetAllColors(CColor::GREEN);
      m_sStateData.State = SStateData::STATE_EXPLORING;
      m_sStateData.TimeRested = 0;
   }
   else {
      ++m_sStateData.TimeRested;

      if(m_sStateData.TimeRested == 1) {
         m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
      }
      /*
       * Social rule: listen to what other people have found and modify
       * probabilities accordingly
       */
      const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
      for(size_t i = 0; i < tPackets.size(); ++i) {
         switch(tPackets[i].Data[0]) {
            case LAST_EXPLORATION_SUCCESSFUL: {
               m_sStateData.RestToExploreProb += m_sStateData.SocialRuleRestToExploreDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
               m_sStateData.ExploreToRestProb -= m_sStateData.SocialRuleExploreToRestDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
               break;
            }
            case LAST_EXPLORATION_UNSUCCESSFUL: {
               m_sStateData.ExploreToRestProb += m_sStateData.SocialRuleExploreToRestDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
               m_sStateData.RestToExploreProb -= m_sStateData.SocialRuleRestToExploreDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
               break;
            }
         }
      }
   }
}

// estado de exploração, robôs ativamente procuram pelo alvo
// evitando colisão com : 1) outros robôs e 2) paredes
// todo : implementar ParticleSwarmOptimization

void FootBotTrack::Explore() {

   bool bFoundTarget(false);

   if(m_Alvo.AlvoSpotted) {

      m_sStateData.ExploreToRestProb -= m_sStateData.FoodRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
      bFoundTarget = true;
   }

   else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
      if (m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.ExploreToRestProb) {
         m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
         bFoundTarget = true;
      }
      else {

         m_sStateData.ExploreToRestProb += m_sStateData.FoodRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
   }

   if(bFoundTarget) {

      m_sStateData.TimeExploringUnsuccessfully = 0;
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_pcLEDs->SetAllColors(CColor::BLUE);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   }
   else {

      ++m_sStateData.TimeExploringUnsuccessfully;
      UpdateState();
      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);
      if(bCollision) {

         m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }

      if(m_sStateData.InNest) {

         SetWheelSpeedsFromVector(
            m_sWheelTurningParams.MaxSpeed * cDiffusion -
            m_sWheelTurningParams.MaxSpeed * 0.25f * CalculateVectorToLight());
      }
      else {

         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
   }
}


void FootBotTrack::FoundTarget() {
   UpdateState();
   if(m_sStateData.InNest) {
      if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
         m_pcRABA->SetData(0, m_eLastExplorationResult);
         m_pcLEDs->SetAllColors(CColor::RED);
         m_sStateData.State = SStateData::STATE_RESTING;
         m_sStateData.TimeSearchingForPlaceInNest = 0;
         m_eLastExplorationResult = LAST_EXPLORATION_NONE;
         return;
      }
      else {
         ++m_sStateData.TimeSearchingForPlaceInNest;
      }
   }
   else {
      m_sStateData.TimeSearchingForPlaceInNest = 0;
   }

   SetWheelSpeedsFromVector(CVector2(0.0, 0.0));
}


REGISTER_CONTROLLER(FootBotTrack, "footbot_foraging_controller")
