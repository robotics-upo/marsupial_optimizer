
#include "marsupial_g2o/catenary_misc.h"


CatenaryMisc::CatenaryMisc(){
	_v_states_catenary = new vector<structCatenary>(20);
	_v_states_catenary->at(0).count_ = 0;
	_v_states_catenary->at(0).count_negative_z_ = 0;
	_v_states_catenary->at(0).d_average_collision_ = 0.0;
	_v_states_catenary->at(0).l_catenary_ = 0.0;
}

void CatenaryMisc::SetValueCount(int pos, int value){
	_v_states_catenary->at(pos).count_ = value;
}

void CatenaryMisc::SetValueCountNegativeZ(int pos, int value){
	_v_states_catenary->at(pos).count_negative_z_ = value;
}

void CatenaryMisc::SetValueLengthCatenary(int pos, double value){
	_v_states_catenary->at(pos).d_average_collision_ = value;
}

void CatenaryMisc::SetValueDistanceAverage(int pos, double value){
	_v_states_catenary->at(pos).l_catenary_ = value;
}
