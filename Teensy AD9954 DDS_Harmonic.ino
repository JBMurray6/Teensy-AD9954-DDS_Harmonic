#include <SPI.h>
#include "SerialFuncInterface.h"
#include "AD9954.h"
#include <arm_math.h>

#define CSPIN  10
#define RESETPIN  9
#define UPDATEPIN  8
#define PS0PIN  7
#define PS1PIN  6
#define OSKPIN  5

#define DDSCLOCK_HZ 20000000
#define DDSCLOCK_MULT 20
#define DDSFREQ_HZ DDSCLOCK_HZ*DDSCLOCK_MULT
#define DDS_MINTIME_ms 1000*4/((double)DDSFREQ_HZ)
#define DDS_MINRATE_ms DDSFREQ_HZ/4/((double)0xFFFFFFFF)/(DDS_MINTIME_ms*255)



struct ReadingStruct
{
	float Freq;
	prog_uint16_t Gain;
	prog_uint16_t Phase;
};

typedef struct  ReadingStruct ReadingStruct;

const int MaxSavesMask = 0x7FF;
ReadingStruct ReadingSaveRing[MaxSavesMask + 1];
int SaveCount = 0;

float CurrSetFreq = 1000;

AD9954 FuncGen = AD9954(CSPIN, RESETPIN, UPDATEPIN, PS0PIN, PS1PIN, OSKPIN);
SPISettings SPIset = SPISettings(1000000, MSBFIRST, SPI_MODE0);

SerialFuncInterfaceClass SerialFuncInterface = SerialFuncInterfaceClass(10);

const unsigned int GAIN_ADC_PIN = 15;
const unsigned int PHASE_ADC_PIN = 14;



enum CurrentLoopMode
{
	Idle = 0,
	Sweep = 1,
	FindPeak = 1
};

typedef struct NameFuncCombo NameFuncCombo;

#pragma region funparameters  


const String FreqPrefix = "Freq";
String GetFreq(String * S);
String SetFreq(String * S);
NameFuncCombo FreqFuncs = { FreqPrefix,SetFreq,GetFreq };

const String AmplitudePrefix = "Amp";
String GetAmplitude(String * S);
String SetAmplitude(String * S);
NameFuncCombo AmplitudeFuncs = { AmplitudePrefix,SetAmplitude,GetAmplitude };
float Amplitude = 1;

const String DACAmplitudePrefix = "DACAmp";
String DACGetAmplitude(String * S);
String DACSetAmplitude(String * S);
NameFuncCombo DACAmplitudeFuncs = { DACAmplitudePrefix,DACSetAmplitude,DACGetAmplitude };
float DACAmplitude = 1;
const unsigned int DACPin = A14;
const unsigned int Resolution = 12;
const float MaxDACInt = (float)((1 << 12) - 1);

///Sweep variables
const String FreqSweepPrefix = "Sweep";
String GetFreqSweep(String * S);
String SetFreqSweep(String * S);
NameFuncCombo FreqSweepFuncs = { FreqSweepPrefix,SetFreqSweep,GetFreqSweep };
enum SweepTypes
{
	LIN = 0,
	LOG = 1
};
const int NumSweepParams = 4;
const float initfloat = 0;
const TypedParameter FreqSweepParams[NumSweepParams] = {
	{ initfloat , FloatVar,  true },//Start
{ initfloat , FloatVar, true },//Stop
{ (int)1000 , IntVar, false },//Num of milliseconds
{ (int)0 , IntVar, false }//SweepType enum
};
//Used to hold sweep as it was most recently set
TypedParameter LastSetSweep[NumSweepParams];
//Manipulated during sweep 
TypedParameter CurrentSweep[NumSweepParams];
bool SweepGoing = false;
bool SweepMode_Cont = false;

const String SweepModePrefix = "SwMode";
String GetSweepMode(String * S);
String SetSweepMode(String * S);
NameFuncCombo SweepModeFuncs = { SweepModePrefix,SetSweepMode,GetSweepMode };

/// Harmonic search variables
const String SearchHamronicsPrefix = "Search";
String GetSearchHamronicsSweep(String * S);
String SetSearchHamronicsSweep(String * S);
NameFuncCombo SearchHamronicsFuncs = { SearchHamronicsPrefix,SetSearchHamronicsSweep,GetSearchHamronicsSweep };
const int NumSearchHamronicsParams = 2;
const TypedParameter SearchHamronicsParams[NumSearchHamronicsParams] = {
	{ {.fval = 4990000} , FloatVar,  false },//Center
	{ {.fval = 6000}, FloatVar, false },//Span
};
//Used to hold sweep as it was most recently set
TypedParameter LastSearchHamronicsSweep[NumSweepParams];
//Manipulated during sweep 
TypedParameter SearchHamronicsSweep[NumSweepParams];
bool SearchGoing = false;
const int NumHarms = 4;
unsigned int HarmCount = 0;
float HarmFreq[NumHarms];
const unsigned int SmoothMask = 0x07;//will create length 8 smoothing
float LastNFreq[SmoothMask + 1];
unsigned int CurrBuffPos = 0;
float SmoothResp = 0;
float CurrBestFreq = 0;
float CurrBestResp = 0;

///Setup functions 
const String ReInitPrefix = "ReInit";
String SetReInit(String * S);
NameFuncCombo ReInitFuncs = { ReInitPrefix,SetReInit,NULL };

#pragma endregion

prog_uint32_t StartMillisecods = 0;
float SweepRate;

void setup()
{
	analogWriteResolution(Resolution);
	pinMode(DACPin, OUTPUT);
	analogReadRes(13);
	pinMode(GAIN_ADC_PIN, INPUT);
	pinMode(PHASE_ADC_PIN, INPUT);

	SPI.begin();
	SPI.beginTransaction(SPIset);

	FuncGen.initialize(DDSCLOCK_HZ, DDSCLOCK_MULT);
	FuncGen.setFreq(1000000);
	SerialFuncInterface.AddFunc(FreqFuncs);
	SerialFuncInterface.AddFunc(AmplitudeFuncs);
	SerialFuncInterface.AddFunc(FreqSweepFuncs);
	SerialFuncInterface.AddFunc(SweepModeFuncs);
	SerialFuncInterface.AddFunc(ReInitFuncs);
	SerialFuncInterface.AddFunc(DACAmplitudeFuncs);
	SerialFuncInterface.AddFunc(SearchHamronicsFuncs);
	Serial.println((DDS_MINTIME_ms * 255));

}

void loop()
{

	/* add main program code here */
	SerialFuncInterface.ParseSerial();

	if (SweepGoing)
	{
		long del = millis() - StartMillisecods;


		FuncGen.setFreq((long)CurrSetFreq);


		ReadingSaveRing[SaveCount&MaxSavesMask].Gain = analogRead(GAIN_ADC_PIN);
		ReadingSaveRing[SaveCount&MaxSavesMask].Freq = CurrSetFreq;
		ReadingSaveRing[SaveCount&MaxSavesMask].Phase = analogRead(PHASE_ADC_PIN);


		if (SearchGoing)
		{
			LastNFreq[CurrBuffPos&SmoothMask] = ReadingSaveRing[SaveCount&MaxSavesMask].Gain;

			CurrBuffPos++;
			SmoothResp = SmoothResp + ReadingSaveRing[SaveCount&MaxSavesMask].Gain - LastNFreq[(CurrBuffPos - 8)&SmoothMask];

			if (SmoothResp > CurrBestResp)
			{
				CurrBestResp = SmoothResp;
				CurrBestFreq = CurrSetFreq;
			}
		}

		Serial.println(String(CurrSetFreq) + ", " + String(ReadingSaveRing[SaveCount&MaxSavesMask].Gain) + ", " + String(ReadingSaveRing[SaveCount&MaxSavesMask].Phase) + ", " + String(SmoothResp));

		SaveCount++;
		if (CurrentSweep[2].Param.ival < del)//time
		{
			CurrSetFreq = CurrentSweep[1].Param.fval;//make sure to hit the last val
			if (SweepMode_Cont ||(SearchGoing &&(HarmCount<NumHarms)))
			{
				memcpy(&CurrentSweep, &LastSetSweep, sizeof FreqSweepParams);
				if (SearchGoing)
				{
					HarmFreq[HarmCount] = CurrBestFreq;

					SweepRate = SweepRate / (HarmCount * 2 + 1);
					HarmCount++;			
					CurrentSweep[0].Param.fval = CurrentSweep[0].Param.fval * (HarmCount * 2 + 1);//Start
					CurrentSweep[1].Param.fval = CurrentSweep[1].Param.fval * (HarmCount * 2 + 1);//Stop
					SweepRate = SweepRate * (HarmCount * 2 + 1);
					

					Serial.println("Peak at: " + String(CurrBestFreq));

					for (unsigned int i = 0; i < (SmoothMask + 1); i++)
					{
						LastNFreq[i] = 0;
					}
					CurrBuffPos = 0;
					SmoothResp = 0;
					CurrBestFreq = 0;
					CurrBestResp = 0;
					
				}
				else
				{
					
				}				
				StartMillisecods = millis();
				/*Serial.println(String(CurrentSweep[0].Param.fval));*/

			}
			else
			{
				if (SearchGoing)
				{
					Serial.println("Peak at: " + String(CurrBestFreq));
				}
				SweepGoing = false;
				SearchGoing = false;

			}
		}
		else
		{
			//Serial.println(CurrentSweep[3].Param.ival);
			switch (CurrentSweep[3].Param.ival)
			{
			case (SweepTypes::LIN):
				CurrSetFreq = CurrentSweep[0].Param.fval + ((float)del)*(SweepRate);
				break;
			case (SweepTypes::LOG):
				CurrSetFreq = (float)pow(10.0, ((float)del)*(SweepRate)+log10(CurrentSweep[0].Param.fval));
				break;
			default:
				break;
			}
		}

	}
}


String GetFreq(String *)
{
	return String(FuncGen.getFreq());
}

String SetFreq(String * S)
{
	float F = S->toFloat();
	FuncGen.setFreq(F);
	return "Set to " + String(FuncGen.getFreq());
}

String GetAmplitude(String *)
{
	return String(Amplitude);
}

String SetAmplitude(String * S)
{
	Amplitude = S->toFloat();
	if (Amplitude > 1)
	{
		Amplitude = 1;
	}
	else if (Amplitude < 0)
	{
		Amplitude = 0;
	}
	//analogWrite(DACPin, (int)((1 - Amplitude)*MaxDACInt));
	FuncGen.Amplitude((prog_uint16_t)(Amplitude*(float)(0x3FFF)));
	return "Set to " + String(Amplitude);
}

String DACGetAmplitude(String *)
{
	return String(DACAmplitude);
}

String DACSetAmplitude(String * S)
{
	DACAmplitude = S->toFloat();
	if (DACAmplitude > 1)
	{
		DACAmplitude = 1;
	}
	else if (DACAmplitude < 0)
	{
		DACAmplitude = 0;
	}
	analogWrite(DACPin, (int)((1 - DACAmplitude)*MaxDACInt));
	//FuncGen.Amplitude((prog_uint16_t)(DACAmplitude*(float)(0x3FFF)));
	return "Set to " + String(DACAmplitude);
}
//Format: Sweep [Start],[Stop],[SweepTime],[LogSweep=1,LinSweep=0];
String SetFreqSweep(String * S)
{
	memcpy(&LastSetSweep, &FreqSweepParams, sizeof FreqSweepParams);

	String S2 = *S;
	Serial.println(S2);
	Serial.println(*S);

	if (!SerialFuncInterface.ParseArguments(LastSetSweep, NumSweepParams, &(S->append(';'))))
	{
		Serial.println("Failed");
	}

	//StartMillisecods = millis();

	float SweepMillis = LastSetSweep[2].Param.ival;
	float SweepDelta = (float)(LastSetSweep[1].Param.fval - LastSetSweep[0].Param.fval);
	SweepRate = SweepDelta / ((float)SweepMillis);

	//if (SweepRate < (float)DDS_MINRATE_ms)//need to run software sweep
	if (true)
	{
		StartMillisecods = millis();


		switch (LastSetSweep[3].Param.ival)
		{
		case (SweepTypes::LIN):
			//SweepRate = SweepDelta / ((float)LastSetSweep[2].Param.ival);//delta freq/time

			Serial.println((float)SweepRate, 15);

			break;
		case (SweepTypes::LOG):
			SweepRate = ((float)(log10(LastSetSweep[1].Param.fval) - log10(LastSetSweep[0].Param.fval))) / ((float)LastSetSweep[2].Param.ival);//delta freq/time
			Serial.println(String((log10(LastSetSweep[1].Param.fval) - log10(LastSetSweep[0].Param.fval)) / ((float)LastSetSweep[2].Param.ival), 10));
			//Serial.println(String((float)CurrentSweep[2].Param.ival));
			break;
		default:
			return "Not a sweep type";
		}

		SweepGoing = true;
		CurrSetFreq = LastSetSweep[0].Param.fval;
		memcpy(&CurrentSweep, &LastSetSweep, sizeof FreqSweepParams);
		return "Runing software sweep";
	}
	else //can run hardware sweep
	{

		float StepSize = (SweepDelta / (SweepMillis / ((float)DDS_MINTIME_ms)) + 1);
		switch (LastSetSweep[3].Param.ival)
		{
		case (SweepTypes::LIN):
			FuncGen.linearSweep(LastSetSweep[0].Param.fval, LastSetSweep[1].Param.fval, 1, 255, 1, 255);
			Serial.println("Set");
			break;
		case (SweepTypes::LOG):
			break;
		default:
			return "Not a sweep type";
		}
		return "Runing hardware sweep";
	}
}

String GetFreqSweep(String * S)
{
	return String(CurrentSweep[0].Param.fval);
}

String SetReInit(String * S)
{
	FuncGen.reset();
	FuncGen.initialize(DDSCLOCK_HZ, DDSCLOCK_MULT);
	FuncGen.setFreq(1000000);

	return "Re-Initialized";
}

String GetSweepMode(String * S)
{
	return String(SweepMode_Cont);
}

String SetSweepMode(String * S)
{
	SweepMode_Cont = S->toInt();
	return "Set to " + String(SweepMode_Cont);
}



//Format: Sweep [Start],[Stop],[SweepTime],[LogSweep=1,LinSweep=0];
String SetSearchHamronicsSweep(String * S)
{

	memcpy(&LastSearchHamronicsSweep, &SearchHamronicsParams, sizeof NumSearchHamronicsParams);



	if (!SerialFuncInterface.ParseArguments(LastSearchHamronicsSweep, NumSearchHamronicsParams, &(S->append(';'))))
	{
		Serial.println("Failed");
	}

	//StartMillisecods = millis();
	float SweepDelta = LastSearchHamronicsSweep[1].Param.fval;
	float SweepMillis = 6.0 * SweepDelta / 20.0;//~6 millisec per step and 20 Hz steps



	SearchGoing = true;
	String SSweep = String(LastSearchHamronicsSweep[0].Param.fval - LastSearchHamronicsSweep[1].Param.fval / 2) + ',' +
		String(LastSearchHamronicsSweep[0].Param.fval + LastSearchHamronicsSweep[1].Param.fval / 2) + ',' + String(SweepMillis);

	Serial.println(SSweep);
	SetFreqSweep(&SSweep);

	for (unsigned int i = 0; i < (SmoothMask + 1); i++)
	{
		LastNFreq[i] = 0;
	}
	CurrBuffPos = 0;
	SmoothResp = 0;
	CurrBestFreq = 0;
	CurrBestResp = 0;
	HarmCount = 0;

	return "Started";
}

String GetSearchHamronicsSweep(String * S)
{
	return String(CurrentSweep[0].Param.fval);
}