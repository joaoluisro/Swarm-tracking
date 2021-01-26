#include "foraging_qt_user_functions.h"
#include <footbot_tracking/footbot_tracking.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,CFootBotEntity>(&CForagingQTUserFunctions::Draw);
}

void CForagingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   FootBotTrack& cController = dynamic_cast<FootBotTrack&>(c_entity.GetControllableEntity().GetController());
   FootBotTrack::Alvo& Alvo = cController.GetInfoAlvo();
   if(Alvo.AlvoSpotted) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f),
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}


REGISTER_QTOPENGL_USER_FUNCTIONS(CForagingQTUserFunctions, "foraging_qt_user_functions")
