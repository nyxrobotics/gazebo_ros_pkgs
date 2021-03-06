#include <gazebo_ros_control/motor_model.h>

MotorModel::MotorModel(MotorElectricCharacteristics motor_electrics,
		MotorHeatCharacteristics motor_heats,
		HeatValues heats){
	default_electric_characteristics_ = motor_electrics;
	heat_characteristics_ = motor_heats;
	heat_values_ = heats;
	heat_values_.motor_temperature = heat_values_.motor_temperature_default;
	use_heat_sim_ = true;
}
MotorModel::MotorModel(MotorElectricCharacteristics motor_electrics){
	default_electric_characteristics_ = motor_electrics;
	use_heat_sim_ = false;
}
MotorModel::MotorModel(){
}
MotorModel::~MotorModel(){
}
void MotorModel::setMotorModel(MotorElectricCharacteristics motor_electrics){
	default_electric_characteristics_ = motor_electrics;
	use_heat_sim_ = false;
}
double MotorModel::duty2Torque(double Duty){
	double torque = 0.;
	if(use_heat_sim_ == false){
		torque = Duty * default_electric_characteristics_.motor_stall_torque_;
	}
	return torque;
}

double MotorModel::duty2Torque(double Duty, double Speed){
	double torque = 0.;
	double speed_tmp = Speed;
	if(speed_tmp > default_electric_characteristics_.motor_noload_speed_){
		speed_tmp = default_electric_characteristics_.motor_noload_speed_;
	}else if(speed_tmp < -default_electric_characteristics_.motor_noload_speed_){
		speed_tmp = -default_electric_characteristics_.motor_noload_speed_;
	}

	if(use_heat_sim_ == false){
		if(Duty > 0.0001){
			torque = default_electric_characteristics_.motor_stall_torque_ * (sqrt(fabs(Duty)) - (speed_tmp/default_electric_characteristics_.motor_noload_speed_));
//			if(torque > sqrt(fabs(Duty)) * default_electric_characteristics_.motor_stall_torque_){
//				torque = sqrt(fabs(Duty)) * default_electric_characteristics_.motor_stall_torque_;
//			}
//			if(default_electric_characteristics_.viscocity_ > 0.0001 && speed_tmp >  0.0001){
//				torque -= speed_tmp * default_electric_characteristics_.viscocity_;
//			}
			if(torque < 0.0){
				torque = 0.0;
			}
		}else if(Duty < -0.0001){
			torque = default_electric_characteristics_.motor_stall_torque_ * -(sqrt(fabs(Duty)) - (speed_tmp/default_electric_characteristics_.motor_noload_speed_));
//			if(torque < -sqrt(fabs(Duty)) * default_electric_characteristics_.motor_stall_torque_){
//				torque = -sqrt(fabs(Duty)) * default_electric_characteristics_.motor_stall_torque_;
//			}
//			if(default_electric_characteristics_.viscocity_ > 0.0001 && speed_tmp < -0.0001){
//				torque -= speed_tmp * default_electric_characteristics_.viscocity_;
//			}
			if(torque > 0.0){
				torque = 0.0;
			}
		}else{
			torque = 0.0;
		}

//		speed_tmp = Speed;
//		if(speed_tmp > 10.0){
//			speed_tmp = 10.0;
//		}else if(speed_tmp < -10.0){
//			speed_tmp = -10.0;
//		}
		torque -= speed_tmp * default_electric_characteristics_.viscocity_;

	}
	return torque;
}

