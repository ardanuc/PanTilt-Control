#define DEBUGFLAG 1

// Flag to skip Low pass filter procesing to speed up the main feedback loop
#define PERFORM_LPF_PROCESSING 1
// This is for different sensor calibrations
# define MAG_SENSOR_ID 2

// AXIS LABELS
enum AxisLabelType {AZIM_AXIS = 1, ELEV_AXIS = 2};

//function prototypes
void TurnOffSelectedMirror();

/* Minimum values of AXIS
First axis maximum/minimum is azimuthal limitsof the dead zone
Second axis maximum/minimum is tilt lmits in milig
For angular deadzones with a periodicity of 360, detemination of whether you
are in the deadzone or not is explained is explained in Suntomics
Research notebook
dated 06/27/2014, page 20

For angular variables, the range of motion is divided into deadzone and activezone which
are complementary and whose sum spans the total 360 degrees.

EXT1 and EXT2 are not random
I will assume that
AxDeadZone_EXT1 (shorthand EXT1) :
The angular value between 0 and 360 which is the
which is the first boundary you hit if you rotate clockwise, when you are in the active region

AxDeadZone_EXT2 (shorthand EXT2) :
The angular value between 0 and 360 which is the
which is the first boundary you hit if you rotate ANTIclockwise, when you are in the active region

Note that within the active region EXT2 will always be in your counterclockwise (negative sense)
and EXT1 will be in your clockwise (positive) direction

---
For linear variables such as acceleration,
assume EXT1 is the minimum vaue of te sensor reading and EXT2 is the maximum

*/

// All angles are in units of 0.01 degrees
/* For AxDeadZone_EXT(1,2) and AxFeedbackEpsilon 
each column represents a different sensor asociated with axis

Column-1 : AZIMUTH axis using magnetic heading function
           --angular, 36000 (360 degrees) periodicity, long
Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
           --linear
Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
           --linear
Column-4 : ELEVATION axis using potentiometer ENCODER Reading
           --linear
*/

#if MAG_SENSOR_ID==1
const int AxDeadZone_EXT1[] = {400, -780, 30, 382};
const int AxDeadZone_EXT2[] = {2900, 780, 1020, 687} ;
#endif


#if MAG_SENSOR_ID==2
const int AxDeadZone_EXT1[] = {400, -780, 30, 382};
const int AxDeadZone_EXT2[] = {2900, 780, 1020, 687} ;
#endif

/*
// error correction residual during the feedback
// Units are 

Column-1 : AZIMUTH axis using magnetic heading function
           --angular, Units 0.01 degrees
Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
           --linear -UNIT: 1 mg
Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
           --linear -UNIT: Least Significant Bi
Column-4 : ELEVATION axis using potentiometer ENCODER Reading
           --linear -UNIT: Least Significant Bit
*/
const int AxFeedbackEpsilon[] = {1500, 10, 0, 0};

//keep converting from float to degrees

// move to a Target Location
// This function
//TargetAxisLabel is an enum of AxisLabelType Class taking in values of AZIM_AXIS and ELEV_AXIS
// TargetReading is the deisted Target Angle in degrees
// ReadFunctionPt is the pointer to the function that returns an integer value for
// MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
// likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

void movetoTarget_1D_angular ( const short & axisIndex, const int TargetReading,
                               long  (*ReadFunctionPt) (void),
                               void (*MovePositiveFunctionPt) (void),
                               void (*MoveNegativeFunctionPt) (void) )
{
  static long CurrentReading;
  
  // Low Pass filtered version
  static long CurrentReading_LP;
  // Low pass moving average numberof samples
  static const short Nsamples_LP = 5;
  static long data_array_LP[Nsamples_LP];
  // temporary variable to store the sum of the variables
  static long temp_LP = 0;
  //index for LP filter
  static short index_LP;

  // Realizable Target Reading based on the dead zone limits
  static long TargetReading_R;
  // The angular extremes of the deadzone
  static long ext_1, ext_2, epsilon_axis;
  ext_1 = AxDeadZone_EXT1[axisIndex] % 36000L;
  ext_2 = AxDeadZone_EXT2[axisIndex] % 36000L;
  epsilon_axis = AxFeedbackEpsilon[axisIndex];

  // flag to indicate dead zone
  static bool flag_dead_zone;
  //initialize to false initially
  flag_dead_zone = false;

  //Read the current value
  CurrentReading = ReadFunctionPt();
  // Look at the target value and try to go to the closerl

  // First determine if TargetReading  is in the deadzone or not
  // analyze in two cases based on the modulus 360 change
  if (ext_2 > ext_1)
  { // This is the regular case
    flag_dead_zone = (TargetReading > ext_1) && (TargetReading < ext_2);
  }
  else
  { // This is the case when there is a modulus change between ext_2 and ext_@
    flag_dead_zone = !((TargetReading > ext_2) && (TargetReading < ext_1));
  }

  // If the target is in the dead zone, it is also desired to move tothe closest
  // extreme point. Since I am using integermodula operator, accuracy is to within a degree


  if (!flag_dead_zone)
  {
    // not in dead zone, so no problem
    TargetReading_R = TargetReading % 36000;
  }

  else
  {
    TargetReading_R = (((int)(ext_2 - TargetReading) % 36000L) > ((int)(TargetReading - ext_1) % 36000L))
                      ? ext_1 : ext_2;
  }

  //Read the current value
  CurrentReading = ReadFunctionPt() % 36000;

  //initialize the low pass filtered version with this inital reading

  for (index_LP = 0; index_LP < Nsamples_LP; index_LP++)
  {
    data_array_LP[index_LP] = CurrentReading;
  }

  //initialize the sum
  temp_LP = Nsamples_LP * CurrentReading;
 
  // update the low passed version
   CurrentReading_LP=CurrentReading;

  // Look at the target value and try to go to the closerl


  // main feedback loop
  #if DEBUGFLAG
  Serial.println("Moving: START!");
 
 #endif
 
  while (abs(CurrentReading - TargetReading_R) > epsilon_axis)
  {
    if 
    (
    ((CurrentReading <= TargetReading_R)&& 
    ((ext_1>=TargetReading_R)||(ext_1<=CurrentReading)))
    ||
    ((CurrentReading>=ext_1)&& (ext_1>=TargetReading))
    ||
    ((CurrentReading>=TargetReading_R)&& (ext_1>=TargetReading_R))
    )

    {
      #if DEBUGFLAG
      Serial.print("Running positive function");
      #endif
      MovePositiveFunctionPt();
    }
    else
    {
      #if DEBUGFLAG
      Serial.print("Running negative function");
      #endif
      MoveNegativeFunctionPt();
    }

    //Read the current value
    CurrentReading = ReadFunctionPt()%36000;



    // Update the Low pass version
    index_LP = (index_LP + 1) % Nsamples_LP;
    temp_LP += CurrentReading - data_array_LP[index_LP];
    data_array_LP[index_LP] = CurrentReading;

    CurrentReading_LP = (temp_LP / (long)Nsamples_LP);

#if DEBUGFLAG
    Serial.print(" ACTU: ");
    Serial.print(CurrentReading);
    Serial.print(" LP: ");
    Serial.print(CurrentReading_LP);
    Serial.print(" Target: ");
    Serial.print(TargetReading);
    Serial.print(" Target-R: ");
    Serial.print(TargetReading_R);
    Serial.print(" ext-1: ");
    Serial.print(ext_1);
    Serial.print(" ext-2: ");
    Serial.println(ext_2);


#endif


  }



#if DEBUGFLAG


  Serial.println("Moving: DONE!");

#endif

  //Since we are done-- turn off the motor
  TurnOffSelectedMirror();


}


// move to a Target Location,


// TargetReading is the desired Sensor Reading,whic can be in milig
// ReadFunctionPt is the pointer to the function that returns an integer value for
// MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
// likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

void movetoTarget_1D_linear ( const short & axisIndex, const int TargetReading,
                              int  (*ReadFunctionPt) (void),
                              void (*MovePositiveFunctionPt) (void),
                              void (*MoveNegativeFunctionPt) (void) )
{
  static int CurrentReading;
 
  // Low Pass filtered version
  static int CurrentReading_LP;
  // Low pass moving average numberof samples
  static const short Nsamples_LP = 5;
  static int data_array_LP[Nsamples_LP];
  // temporary variable to store the sum of the variables
  static long temp_LP = 0;
  //index for LP filter
  static short index_LP;

  // Realizable Target Reading based on the dead zone limits
  static int TargetReading_R;
  // The angular extremes of the deadzone
  static int ext_1, ext_2, epsilon_axis;
  ext_1 = AxDeadZone_EXT1[axisIndex];
  ext_2 = AxDeadZone_EXT2[axisIndex];
  epsilon_axis = (int) AxFeedbackEpsilon[axisIndex];

  // flag to indicate dead zone
  static bool flag_dead_zone;
  //initialize to false initially
  flag_dead_zone = false;


  // First determine if TargetReading  is in the deadzone or not
  // analyze in two cases based on the modulus 360 change
  if (!(ext_2 >= ext_1))
  { // in case ext_2 is not entered as the maximum , swap the variables
    ext_2 = AxDeadZone_EXT1[axisIndex];
    ext_1 = AxDeadZone_EXT2[axisIndex];

  }


  flag_dead_zone = (TargetReading < ext_1) || (TargetReading > ext_2);

  // If the target is in the dead zone, it is also desired to move tothe closest
  if (!flag_dead_zone)
  {
    // not in dead zone, so no problem
    TargetReading_R = TargetReading;
  }

  else
  {
    TargetReading_R = (TargetReading < ext_1) ? ext_1 : ext_2;
  }

  //Read the current value
  CurrentReading = ReadFunctionPt();
  


  //initialize the low pass filtered version with this inital reading

  for (index_LP = 0; index_LP < Nsamples_LP; index_LP++)
  {
    data_array_LP[index_LP] = CurrentReading;
  }

  //initialize the sum
  temp_LP = Nsamples_LP * CurrentReading;

  // update the low passed version
   CurrentReading_LP=CurrentReading;
  // Look at the target value and try to go to the closerl

  // main feedback loop
  #if DEBUGFLAG
  Serial.println("Moving!");

 #endif
 
  while (abs(CurrentReading - TargetReading_R) > epsilon_axis)
  {
    if (CurrentReading >= TargetReading_R)
      {
      #if DEBUGFLAG
      Serial.print("Running negative function");
      #endif
      MoveNegativeFunctionPt();
      }
    else
    {
      #if DEBUGFLAG
      Serial.print("Running positive function");
      #endif
      MovePositiveFunctionPt();
    }
    //Read the current value
    CurrentReading = ReadFunctionPt();


#if PERFORM_LPF_PROCESSING ==1

    // Update the Low pass version
    index_LP = (index_LP + 1) % Nsamples_LP;
    temp_LP += CurrentReading - data_array_LP[index_LP];
    data_array_LP[index_LP] = CurrentReading;

    CurrentReading_LP = (int)(temp_LP / (long)Nsamples_LP);
    
#endif    

#if DEBUGFLAG
    Serial.print(" ACTU: ");
    Serial.print(CurrentReading);
    Serial.print(" LP: ");
    Serial.print(CurrentReading_LP);
    Serial.print(" Target: ");
    Serial.print(TargetReading);
    Serial.print(" Target-R: ");
    Serial.println(TargetReading_R);


#endif

  }

#if DEBUGFLAG
  Serial.println("Moving: DONE!");
#endif  
  
  //Since we are done-- turn off the motor
  TurnOffSelectedMirror();

  //Serial.println("You made it here");
  //Serial.print(ReadFunctionPt());

}

