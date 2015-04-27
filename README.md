# POLARIS
**Position Oriantation Locatization Augmented Reality (AR) Tag Indoor System**
*******************************************************************************
*******************************************************************************

The Position Orientation Localization ARTag Recognition Indoor System (POLARIS)
is an indoor localization module meant to provide localization functionality to
generic robots indoors. GPS has been utilized for outdoor localization and has
spawned many interesting applications of localization. Such applications of GPS
include emergency response and navigation, GPS is also heavily used by military
systems. However, the line of sight of GPS satellites tend to be blocked 
indoors. Therefore, there is a need for a system that can provide reliable
localization indoors. POLARIS will be created for localization within the GMU
Nguyen Engineering building. POLARIS will allow for localization through 
infrared computer vision of non-salient ARTags. When an ARTag is viewed, it 
will give the device a unique global position based on its glyph ID. This 
unique global position will be from pre-collected data obtained through 
simultaneous localization and mapping (SLAM) to fill a lookup-table (LUT). The
global position will then be utilized along with the 6D pose of the viewed 
ARTag to provide the global frame position of POLARIS.
