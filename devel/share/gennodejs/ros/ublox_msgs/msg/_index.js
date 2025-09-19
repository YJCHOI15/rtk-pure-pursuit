
"use strict";

let NavRELPOSNED = require('./NavRELPOSNED.js');
let MgaGAL = require('./MgaGAL.js');
let EsfINS = require('./EsfINS.js');
let Ack = require('./Ack.js');
let CfgNAV5 = require('./CfgNAV5.js');
let Inf = require('./Inf.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let AidALM = require('./AidALM.js');
let CfgNMEA7 = require('./CfgNMEA7.js');
let NavVELNED = require('./NavVELNED.js');
let MonHW6 = require('./MonHW6.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let NavCLOCK = require('./NavCLOCK.js');
let NavDGPS = require('./NavDGPS.js');
let EsfMEAS = require('./EsfMEAS.js');
let NavVELECEF = require('./NavVELECEF.js');
let NavATT = require('./NavATT.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let RxmRAW = require('./RxmRAW.js');
let NavSVIN = require('./NavSVIN.js');
let TimTM2 = require('./TimTM2.js');
let EsfRAW = require('./EsfRAW.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let CfgCFG = require('./CfgCFG.js');
let RxmSVSI = require('./RxmSVSI.js');
let MonVER = require('./MonVER.js');
let UpdSOS = require('./UpdSOS.js');
let CfgDAT = require('./CfgDAT.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let CfgHNR = require('./CfgHNR.js');
let RxmSFRB = require('./RxmSFRB.js');
let NavSAT = require('./NavSAT.js');
let NavDOP = require('./NavDOP.js');
let CfgNMEA = require('./CfgNMEA.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let MonHW = require('./MonHW.js');
let MonGNSS = require('./MonGNSS.js');
let CfgANT = require('./CfgANT.js');
let AidHUI = require('./AidHUI.js');
let RxmRAWX = require('./RxmRAWX.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let CfgMSG = require('./CfgMSG.js');
let NavSBAS = require('./NavSBAS.js');
let HnrPVT = require('./HnrPVT.js');
let RxmRTCM = require('./RxmRTCM.js');
let NavSOL = require('./NavSOL.js');
let CfgINF = require('./CfgINF.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let CfgSBAS = require('./CfgSBAS.js');
let NavSVINFO = require('./NavSVINFO.js');
let RxmEPH = require('./RxmEPH.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let AidEPH = require('./AidEPH.js');
let RxmALM = require('./RxmALM.js');
let CfgRST = require('./CfgRST.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let CfgRATE = require('./CfgRATE.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let CfgPRT = require('./CfgPRT.js');
let CfgGNSS = require('./CfgGNSS.js');
let NavPVT = require('./NavPVT.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let NavPVT7 = require('./NavPVT7.js');
let CfgUSB = require('./CfgUSB.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let NavSTATUS = require('./NavSTATUS.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');

module.exports = {
  NavRELPOSNED: NavRELPOSNED,
  MgaGAL: MgaGAL,
  EsfINS: EsfINS,
  Ack: Ack,
  CfgNAV5: CfgNAV5,
  Inf: Inf,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  RxmRAW_SV: RxmRAW_SV,
  AidALM: AidALM,
  CfgNMEA7: CfgNMEA7,
  NavVELNED: NavVELNED,
  MonHW6: MonHW6,
  NavTIMEUTC: NavTIMEUTC,
  NavCLOCK: NavCLOCK,
  NavDGPS: NavDGPS,
  EsfMEAS: EsfMEAS,
  NavVELECEF: NavVELECEF,
  NavATT: NavATT,
  CfgDGNSS: CfgDGNSS,
  RxmRAW: RxmRAW,
  NavSVIN: NavSVIN,
  TimTM2: TimTM2,
  EsfRAW: EsfRAW,
  NavSBAS_SV: NavSBAS_SV,
  CfgCFG: CfgCFG,
  RxmSVSI: RxmSVSI,
  MonVER: MonVER,
  UpdSOS: UpdSOS,
  CfgDAT: CfgDAT,
  EsfSTATUS: EsfSTATUS,
  NavPOSECEF: NavPOSECEF,
  CfgHNR: CfgHNR,
  RxmSFRB: RxmSFRB,
  NavSAT: NavSAT,
  NavDOP: NavDOP,
  CfgNMEA: CfgNMEA,
  RxmSFRBX: RxmSFRBX,
  MonHW: MonHW,
  MonGNSS: MonGNSS,
  CfgANT: CfgANT,
  AidHUI: AidHUI,
  RxmRAWX: RxmRAWX,
  CfgGNSS_Block: CfgGNSS_Block,
  CfgMSG: CfgMSG,
  NavSBAS: NavSBAS,
  HnrPVT: HnrPVT,
  RxmRTCM: RxmRTCM,
  NavSOL: NavSOL,
  CfgINF: CfgINF,
  CfgTMODE3: CfgTMODE3,
  NavPOSLLH: NavPOSLLH,
  CfgSBAS: CfgSBAS,
  NavSVINFO: NavSVINFO,
  RxmEPH: RxmEPH,
  NavDGPS_SV: NavDGPS_SV,
  RxmRAWX_Meas: RxmRAWX_Meas,
  AidEPH: AidEPH,
  RxmALM: RxmALM,
  CfgRST: CfgRST,
  RxmSVSI_SV: RxmSVSI_SV,
  CfgRATE: CfgRATE,
  CfgINF_Block: CfgINF_Block,
  NavSAT_SV: NavSAT_SV,
  CfgPRT: CfgPRT,
  CfgGNSS: CfgGNSS,
  NavPVT: NavPVT,
  CfgNMEA6: CfgNMEA6,
  CfgNAVX5: CfgNAVX5,
  NavPVT7: NavPVT7,
  CfgUSB: CfgUSB,
  MonVER_Extension: MonVER_Extension,
  NavSVINFO_SV: NavSVINFO_SV,
  EsfRAW_Block: EsfRAW_Block,
  NavSTATUS: NavSTATUS,
  UpdSOS_Ack: UpdSOS_Ack,
  NavTIMEGPS: NavTIMEGPS,
};
