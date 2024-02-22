using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using NMotive;

namespace MotiveBatchProcessor.Examples
{
    /// <summary>
    /// Motive Batch Processor script for running processing retieval data from Filbury.
    /// </summary>
    public class RetrievalBatch : ITakeProcessingScript
    {
        delegate bool PrintKeyframePredicate(int frameIndex);

        #region Instance Variables -------------------------------------------

        /// <summary>
        /// This variable will hold the argument string that was entered by
        /// the user when the script is run by the batch processor.
        /// </summary>
        private string[] mBatchProcessorArguments;

        #endregion Instance Variables

        #region ITakeProcessingScriptWithArgs interface implementation -------

        /// <summary>
        /// The argument string entered by the user when running the script in
        /// the batch processor.
        /// </summary>
        public string[] Arguments
        {
            set { mBatchProcessorArguments = value; }
        }

        /// <summary>
        /// This string will be placed next to the text box where the
        /// user enters the maximum gap size to fill.
        /// </summary>
        public string ArgumentLabel
        {
            get { return "Enter maximum gap size to fill:"; }
        }

        /// <summary>
        /// The default value.
        /// </summary>
        public string[] DefaultArguments
        {
            get { return new string[0]; }
        }

        /// <summary>
        /// After the user has entered the argument string, the batch processor will
        /// call this function to validate it. In this case we expect an positive
        /// integer value to be entered.
        /// </summary>
        /// <param name="argString">The user entered argument string.</param>
        /// <returns>The result of the validation. If the string represents
        /// a valid integer Result.Success is set to true. If it does not
        /// represent a valid positive integer Result.Success is set to false
        /// and Result.Message contains further information.</returns>
        public Result ValidateArguments(string[] argString)
        {
            int n;

            if (argString.Length >= 1)
            {
                if (int.TryParse(argString[0], out n))
                {
                    if (int.Parse(argString[0]) >= 0)
                    {
                        // Returns true if the input is a positive integer.
                        return new Result(true, "");
                    }
                    else
                    {
                        return new Result(false, argString[0] + " is not a positive integer.");
                    }
                }
                else
                {
                    return new Result(false, argString[0] + " is not a valid integer.");
                }
            }
            return new Result(false, "Too few arguments.");
        }





        /// <summary>
        /// The <c>ProcessTake</c> function is from the <c>ITakeProcessingScript</c> interface.
        /// Creates or reprocesses 3D data for a given take by reconstructing and
        /// auto-labeling it. Updates the loaded take instead of creating a new one.
        /// </summary>
        /// <param name="take">The take to export.</param>
        /// <param name="progress">Progress indicator object. Shows the current take
        /// and what is currently being processed.</param>
        /// <returns>The same file with new 3D data.</returns>
        /// 
        /// /// Performs the gap filling task on the given take.
        /// </summary>
        /// <param name="take">A take.</param>
        /// <param name="progress">Progress indicator.</param>
        /// <returns>The result of gap filling.</returns>
        public Result ProcessTake(Take take, ProgressIndicator progress)
        {
            // Set reconstruction settings to use.
            string fileName = "D:\\nicRobData\\reconstruction_settings_retrieval.mup";
            Result res = Settings.ImportMotiveProfile(fileName);

            if (!res.Success)
            {
                return res;
            }

            // Construct an NMotive Trajectorizer object, then Process the take.
            Trajectorizer traj = new Trajectorizer(progress);
            traj.Process(take, TrajectorizerOption.Reconstruct);
            take.Save();

            // Trim around gaps using smart trim
            TrimTails trimTails = new TrimTails();
            trimTails.Automatic = true;
            trimTails.Process(take);

            // Remove solve, if solved already
            take.RemoveSolve();

            // Autolabel
            traj.Process(take, TrajectorizerOption.AutoLabel);
            take.Save();

            // Linear interpolate small gaps
            int maxGapSize = 10;
            FillGaps fill = new FillGaps(progress);
            fill.InterpolationMode = InterpolationType.Linear; // Constant, Linear, Hermite (Cubic), PatternBased, ModelBased 
            fill.MaxGapFillWidth = maxGapSize;
            Result fillResult = fill.Process(take);

            // Spline interpolate medium gaps
            maxGapSize = 60;
            fill = new FillGaps(progress);
            fill.InterpolationMode = InterpolationType.Hermite; // Constant, Linear, Hermite (Cubic), PatternBased, ModelBased 
            fill.MaxGapFillWidth = maxGapSize;
            fillResult = fill.Process(take);

            // Solve rigid bodies
            take.Solve();

            // Remove unlabelled markers
            List<RealMarker> list = take.Scene.AllMarkers();
            for (int i = 0; list.Count > i; ++i)
            {
                if (list[i].IsLabeled == false)
                {
                    take.Scene.RemoveNode(list[i].Name.ToString());
                }
            }

            // Save
            take.Save();

            // Only interested in the scannercast rigid body
            NodeWarehouse nodeWarehouse = take.Scene;
            List<AnimatedNode> allRigidBodies = nodeWarehouse.AllRigidBodies();

            string newName = "Scannercast";
            if (allRigidBodies.Count == 1)
            {
                take.Scene.AllRigidBodies()[0].Name = newName;

            }
            else
            {
                // Initialize the highest node and its height.
                AnimatedNode highestNode = null;
                float highestHeight = float.MinValue;

                // Iterate through all rigid bodies.
                foreach (AnimatedNode node in allRigidBodies)
                {
                    // Get the height of the current node.
                    float currentHeight = node.Translation(take.FullFrameRange.Center).Z;

                    // If the current node is higher than the highest node so far, update the highest node and its height.
                    if (currentHeight > highestHeight)
                    {
                        highestNode = node;
                        highestHeight = currentHeight;
                    }
                }
                // Rename the highest node.
                if (highestNode != null)
                {
                    highestNode.Name = newName;
                }
            }

            // Disable assets that are not the scannercast
            foreach (AnimatedNode rigidBody in allRigidBodies)
            {
                if (!(rigidBody.Name == newName))
                {
                    rigidBody.Active = false;
                }
            }

            // Filter the data.
            Filter filter = new Filter();
            filter.CutOffFrequency = 8;
            filter.DataRate = take.FrameRate;
            filter.Process(take);

            // Solve rigid bodies again.
            take.Solve();

            // Final save.
            take.Save();

            // Export

            // Construct an NMotive CSV exporter object with the desired
            // options. We will write CSV data for markers and assets.
            CSVExporter exporter = new CSVExporter
            {
                //-== CSVExporter Class ==-
                RotationType = Rotation.QuaternionFormat,   // QuaternionFormat, XYZ, XZY, YXZ, YZX, ZXY, ZYX
                Units = LengthUnits.Units_Millimeters,
                UseWorldSapceCoordinates = true,
                WriteBoneMarkers = false,
                WriteBones = false,
                WriteHeader = true,
                WriteMarkers = true,
                WriteQualityStats = true,
                WriteRigidBodies = true,
                WriteRigidBodyMarkers = true
            };


            // Construct the output CSV file. The output file will be co-located with the
            // take file and have the same name as the take file, but with an '.csv' 
            // file extension and a different BIDS description
            string outputFileName = Path.GetFileNameWithoutExtension(take.FileName);
            int remEnd = 9;
            string newFileDescription = "ScannercastRigidBody" + ".csv";
            // Check if the outputFileName is longer than X
            if (outputFileName.Length > remEnd)
            {
                // Remove the last X characters and add the new file ending
                outputFileName = outputFileName.Substring(0, outputFileName.Length - remEnd) + newFileDescription;
            }
            else
            {
                outputFileName = outputFileName + newFileDescription;
            }

            string outputDirectory = Path.GetDirectoryName(take.FileName);
            string outputFile = Path.Combine(outputDirectory, outputFileName);

            // Do the export and return the Export functions result object.
            progress.SetMessage("Writing to File");
            progress.SetProgress((float)0.1);
            
            return exporter.Export(take, outputFile, true);
        }
        #endregion ITakeProcessingScriptWithArgs interface implementation
    }
}