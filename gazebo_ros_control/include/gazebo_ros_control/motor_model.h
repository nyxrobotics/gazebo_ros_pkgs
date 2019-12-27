#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H
#include <ros/ros.h>

class MotorElectricCharacteristics
{
	public:
		//Default parameter for RS406CB(FUTABA)
		double motor_rated_voltage_ = 11.1;
		double supply_voltage_ = 12.;
		double motor_noload_speed_ = 9.52;//[rad/sec]
		double motor_noload_current_ = 0.30;
		double motor_stall_torque_ = 2.75;
		double motor_stall_current_ = 1.90;
		double viscocity_ = 1.0;//[N*m*sec/rad]
};

class MotorHeatCharacteristics
{
	public:
		double heat_transfer_coefficient = 10.;//[W/(m^2*K)]
		double surface_area = 0.0064; //[m^3]
		double heat_capacity = 0.088; //[J/K] = specific_heat[J/(g*K)] * mass[kg]
		double temperature_coefficient_of_coil_resistance = 0.0039; // default value:copper
		double temperature_coefficient_of_magnetic_force = 0.0019; // default value:ferrite magnet(neodimium -> 0.0013)
};

class HeatValues
{
	public:
		double environment_temperature = 15.; //[celsius]
		double motor_temperature_default = 20.;
		double motor_temperature_min = 20.;
		double motor_temperature_max = 20.;
		double motor_temperature = 20.;
};

class MotorModel {
private:
	MotorElectricCharacteristics default_electric_characteristics_;
	MotorElectricCharacteristics heated_electric_characteristics_;
	MotorHeatCharacteristics heat_characteristics_;
	HeatValues heat_values_;
	bool use_heat_sim_ = false;
public:
	MotorModel(MotorElectricCharacteristics motor_electrics,
			MotorHeatCharacteristics motor_heats,
			HeatValues heats);
	MotorModel(MotorElectricCharacteristics motor_electrics);
	MotorModel();
	~MotorModel();
	void setMotorModel(MotorElectricCharacteristics motor_electrics);
	double duty2Torque(double Duty);
	double duty2Torque(double Duty, double Speed);

private:

};

















#endif
