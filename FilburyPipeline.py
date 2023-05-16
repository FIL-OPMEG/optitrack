import sys
import clr

# Add a reference to the NMotive assembly
clr.AddReference("NMotive")

from System import *
# Import everything from NMotive.
from NMotive import *

import time



# Add a reference to the NMotive assembly
clr.AddReference("NMotive")



def ProcessTake(take, progress):
    # Remove existing solve of assets
    RemoveAllSolve(take)
    
    # Reconstruct data
    ReconstructTake(take, progress)
    
    # Smart trim all markers
    SmartTrim(take, progress)
    
    # Autolabel
    AutolabelMarkers(take, progress)
    
    # Solve assets
    SolveAllAssets(take, progress)
    
    
    
    
    # Model fill gaps
    #ModelFillGaps(take, progress)
    
    
    # Linear fill any remaining gaps
    LinearFillGaps(take, progress)
    
    # Filter data
    HighPassFilter(take, progress)
    
    # Solve assets
    SolveAllAssets(take, progress)
    
    # Save Take
    take.Save()
    
    # Export 
    ExportTakes(take, progress, True)
    ExportTakes(take, progress, False)
    
    
    return Result(True, "All done!")
    
def RemoveAllSolve(take):
    take.RemoveSolve()

    
def ReconstructTake(take, progress):   
   # Construct an NMotive Trajectorizer object, then Process the take.
   reconstruct_and_autolabel = Trajectorizer( progress )
   reconstruct_and_autolabel.Process( take, TrajectorizerOption.Reconstruct )

    
def SmartTrim(take, progress):
   # Create an NMotive TrimTails object to perform the tail trimming operation.
   tailTrimming = TrimTails()
   
   # pass the progress object to the trim tails object. It will update 
   # progress that will be rendered in the UI.
   tailTrimming.Progress = progress
   
   # Set trail trimming options.
   tailTrimming.Automatic = True
   tailTrimming.LeadingTrimSize = 3
   tailTrimming.TrailingTrimSize = 5
   
   # And execute the trimming process.
   trimResult = tailTrimming.Process(take)
    
    
def AutolabelMarkers(take, progress):  
   # Construct an NMotive Trajectorizer object, then Process the take.
   reconstruct_and_autolabel = Trajectorizer( progress )
   reconstruct_and_autolabel.Process( take, TrajectorizerOption.AutoLabel )
    

def SolveAllAssets(take, progress):
   reconstruct_and_autolabel = Trajectorizer( progress )
   reconstruct_and_autolabel.Process( take, TrajectorizerOption.SolveAssets )
    
        
def ModelFillGaps(take, progress):
    # --- Construct a Marker List ---
    # All markers you want pattern filled must have this in the name.
    # Example, rigid body markers are named "Marker1", "Marker2", etc...
    rigidBodyMarkerIdentifier = "Marker"

    # Check to see if the take has markers.
    if len(take.Scene.AllMarkers()) == 0:
        return Result( False, "This Take Does not have marker data.")
    
    # Make a list of IDs of markers that have rigidBodyMarkerIdentifier in them.
    rigidBodyIdList = []
    for marker in take.Scene.AllMarkers():
        if rigidBodyMarkerIdentifier in marker.Name:
            rigidBodyIdList.append(marker.ID)
    
    #--- Run a Pattern Based Gap Fill ---
    # Check to see if the take has gaps.
    if not take.HasGaps():
        return Result(False, "This Take Does not have Gaps.")
    
    # Save Take
    take.Save()
    
    # Variables for the gap fill process.
    fill = FillGaps()
    
    # Then bigger gaps with model fill
    maxGapSize = 10
    fill = FillGaps(maxGapSize)
    for marker_id in rigidBodyIdList:
        # Set the progress indication, then sleep
        prog = float(rigidBodyIdList.index(marker_id)) / float(len(rigidBodyIdList)) * 100
        progress.SetMessage(str(marker_id))
        progress.SetProgress(prog)
        time.sleep(0.1)
        
        # This runs the fill gap on the take.
        fill.InterpolationMode = InterpolationType.ModelBased
        fill.MaxGapFillWidth = maxGapSize
        fillResult = fill.Process(take, [marker_id])
                
        # Reports the success or failure of the FillGap operation.
        if not fillResult.Success:
            # If the gap fill fails do not save the take file.
            return Result(False, "The gap filling process failed for some reason.")
            
    take.Save()
    return Result(True, "Gap filling process completed successfully.")
    
    
    
    
    
def LinearFillGaps(take, progress):
    # Check to see if the take has gaps.
	if not take.HasGaps():
		return Result( False, "This Take Does not have Gaps.")
	
	#Don't fill gaps larger than this size.
	maxGapSize = 10
	
	# This defines a FillGaps type and runs it on the take.
	fill = FillGaps(progress)
	# Constant, Linear, Hermite (Cubic), PatternBased, ModelBased 
	fill.InterpolationMode = InterpolationType.Linear
	fill.MaxGapFillWidth = maxGapSize
	fill.Process(take)

	processResult = Result(False, "The result of this process.")
	processResult = fill.Process(take)

	# Reports the success or failure of the FillGap operation.
	if processResult.Success:
		take.Save()
		return processResult
	else:
		# If the gap fill fails do not save the take file.
		return Result(False, "The gap filling process failed for some reason.")

	return Result(False, "An error occurred.")
    
def HighPassFilter(take, progress):
      # Set the message to be displayed next to to the progress bar in the 
   # Motive Batch Processor UI. 
   # Starting the filtering process...
   progress.SetMessage('Filtering...')
   # Create the NMotive filter object.
   filtering = Filter()
   # We are going to use the progress bar to display the progress of each 
   # individual operation, so reset the progress bar to zero and pass the
   # the progress object to the filtering object.
   progress.SetProgress(0)
   filtering.Progress = progress
   # Set the cutoff frequency and filter.
   filtering.CutOffFrequency = 6 # Hz
   filteringResult = filtering.Process(take)
   if not filteringResult.Success: # If filtering failed, return without saving the take.
      return filteringResult

   # If we get here trimming and filtering succeeded. Save the take file.
   progress.SetMessage('Saving take...')
   fileSaveResult = take.Save()
   if fileSaveResult != FileResult.ResultOK:
      return Result(False, 'File save failed')
   
   return Result(True, '')


def ExportTakes(take, progress,quat):

    # Construct an NMotive CSV exporter object with the desired options.
    exporter = CSVExporter()
    #-== CSVExporter Class ==-
    if quat:
        exporter.RotationType = Rotation.QuaternionFormat     #QuaternionFormat, XYZ, XZY, YXZ, YZX, ZXY, ZYX
        extension = "quat"
    else:
        exporter.RotationType = Rotation.XYZ
        extension = "XYZ"
        
    exporter.Units = LengthUnits.Units_Meters
    exporter.UseWorldSapceCoordinates = True
    exporter.WriteBoneMarkers = False
    exporter.WriteBones = False
    exporter.WriteHeader = True
    exporter.WriteMarkers = False
    exporter.WriteQualityStats = False
    exporter.WriteRigidBodies = True
    exporter.WriteRigidBodyMarkers = True
    #-== Exporter Class ==-
    #exporter.StartFrame = 0
    #exporter.EndFrame = 1000
    #exporter.FrameRate = 120
    #exporter.Scale = 1
    
    # Create a file in the same folder with the same name, but with a different extension.
    full_filename =  take.FileName.strip('.tak')
    outputFile = full_filename + "_allRb_" + extension + ".csv"
    
    # Export the file.
    progress.SetMessage("Writing to File")
    progress.SetProgress( float(0.1) )
    return exporter.Export(take, outputFile, True)


    