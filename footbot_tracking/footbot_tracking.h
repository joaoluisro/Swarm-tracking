#ifndef FOOTBOT_FORAGING_H
#define FOOTBOT_FORAGING_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class FootBotTrack : public CCI_Controller {

public:


   struct Alvo {
      bool AlvoSpotted;      // alvo encontrado
      size_t AlvoID;        // ID do alvo único
      size_t TotalAlvos;    // total de alvos encontrados pelo agente -> máximo é setado para '1'

      Alvo();
      void Reset();
   };

   struct SDiffusionParams {

      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };


   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };


   struct SStateData {
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST
      } State;

      bool InNest;

      Real InitialRestToExploreProb;
      Real RestToExploreProb;
      Real InitialExploreToRestProb;
      Real ExploreToRestProb;
      CRange<Real> ProbRange;
      Real FoodRuleExploreToRestDeltaProb;
      Real FoodRuleRestToExploreDeltaProb;
      Real CollisionRuleExploreToRestDeltaProb;
      Real SocialRuleRestToExploreDeltaProb;
      Real SocialRuleExploreToRestDeltaProb;
      size_t MinimumRestingTime;
      size_t TimeRested;
      size_t MinimumUnsuccessfulExploreTime;
      size_t TimeExploringUnsuccessfully;
      size_t MinimumSearchForPlaceInNestTime;
      size_t TimeSearchingForPlaceInNest;
      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };

public:

   FootBotTrack();
   virtual ~FootBotTrack() {}
   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}


   inline bool IsExploring() const {
      return m_sStateData.State == SStateData::STATE_EXPLORING;
   }

   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_RESTING;
   }

   inline bool IsReturningToNest() const {
      return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
   }


   inline Alvo& GetInfoAlvo() {
      return m_Alvo;
   }

private:

   void UpdateState();
   CVector2 CalculateVectorToLight();
   CVector2 DiffusionVector(bool& b_collision);
   void SetWheelSpeedsFromVector(const CVector2& c_heading);
   void Rest();
   void Explore();
   void ReturnToNest();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;


   CRandom::CRNG* m_pcRNG;

   /* Used in the social rule to communicate the result of the last
    * exploration attempt */
   enum ELastExplorationResult {
      LAST_EXPLORATION_NONE = 0,    // nothing to report
      LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
      LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
   } m_eLastExplorationResult;

   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   Alvo m_Alvo;

};

#endif
