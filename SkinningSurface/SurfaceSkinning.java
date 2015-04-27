package SkinningSurface;

import SpringSolid.Mathematics.math_matrix;
import SpringSolid.Mathematics.math_upoint;
import SpringSolid.Mathematics.math_vector3d;
import SpringSolid.Part42.step_b_spline_curve;
import SpringSolid.Part42.step_b_spline_surface;
import SpringSolid.Part42Array.double_array;
import SpringSolid.Part42Array.math_vector3d_array;
import SpringSolid.Part42Array.step_curve_array;

public class SurfaceSkinning
{
    private step_b_spline_surface skinningSurface;
    private step_b_spline_curve[] sectionCurve;
    private step_b_spline_curve spineCurve;
    private step_b_spline_curve[] guideCurve;
    private math_vector3d[] spineDerivativeVector;
    private math_vector3d[] spineProjectPoint;
    private boolean isSection;
    private boolean isSpine;
    private boolean isGuide;

    public SurfaceSkinning()
    {
        this.isSection = false;
        this.isSpine = false;
        this.isGuide = false;
    }

    public SurfaceSkinning(step_b_spline_curve[] section)
    {
        this.sectionCurve = section;
        this.isSection = true;
        this.isSpine = false;
        this.isGuide = false;
        this.GenerateSurface();
    }

    public SurfaceSkinning(step_b_spline_curve[] section, step_b_spline_curve spine)
    {
        this.sectionCurve = section;
        this.spineCurve = spine;
        this.isSection = true;
        this.isSpine = true;
        this.isGuide = false;
        this.GenerateSurface();
    }

    public SurfaceSkinning(step_b_spline_curve[] section, step_b_spline_curve spine, step_b_spline_curve[] guide)
    {
        this.sectionCurve = section;
        this.spineCurve = spine;
        this.guideCurve = guide;
        this.isSection = true;
        this.isSpine = true;
        this.isGuide = true;
        this.GenerateSurface();
    }

    public void SetSectionCurve(step_b_spline_curve[] section)
    {
        this.sectionCurve = section;
        this.isSection = true;
    }

    public void SetSpineCurve(step_b_spline_curve spine)
    {
        this.spineCurve = spine;
        this.isSpine = true;
    }

    public void SetGuideCurve(step_b_spline_curve[] guide)
    {
        this.sectionCurve = guide;
        this.isSection = true;
    }

    private void GenerateSurface()
    {
        if (isSection && (!isSpine) && (!isGuide))
        {
            SurfaceSkinningWithoutSpine();
        }
        else if (isSection && isSpine && (!isGuide))
        {
            this.SurfaceSkinningWithSpine();
        }
        else if (isSection && isSpine && isGuide)
        {
            SurfaceSkinningWithSpineAndGuide();
        }
    }

    public step_b_spline_surface GetSkinningSurface()
    {
        return skinningSurface;
    }

    private void SurfaceSkinningWithoutSpine()
    {
        //Pre-treatment
        skinningSurface = new step_b_spline_surface();
        PreSetSectionCurve();

        //Define basic information
        int NuControl = sectionCurve[0].Ncontrol();
        int NvControl;
        int uOrder = sectionCurve[0].Order();
        int vOrder;
        step_b_spline_curve[] linkCurve = new step_b_spline_curve[NuControl];

        //Create section curve by connect control points >>>>> linkCurve
        math_vector3d_array[] linkCurveData = new math_vector3d_array[NuControl];
        for (int i = 0; i < NuControl; i++)
        {
            linkCurveData[i] = new math_vector3d_array();
            linkCurve[i] = new step_b_spline_curve();
        }
        for (int i = 0; i < sectionCurve.length; i++)
        {
            for (int j = 0; j < NuControl; j++)
            {
                math_vector3d conVec = sectionCurve[i].GetControlPoint(j).GetCoord();
                linkCurveData[j].Add(conVec);
            }
        }
        for (int i = 0; i < NuControl; i++)
        {
            linkCurve[i].Interpolation(linkCurveData[i]);
        }
        NvControl = linkCurve[0].Ncontrol();
        vOrder = linkCurve[0].Order();

        //Set surface control point
        skinningSurface.SetNucontrol(NuControl);
        skinningSurface.SetNvcontrol(NvControl);
        for (int j = 0; j < NvControl; j++)
        {
            math_upoint up = new math_upoint();
            for (int i = 0; i < NuControl; i++)
            {
                up.AddControlPoint(linkCurve[i].GetControlPoint(j));
            }
            //surface.GetControlPoint().Add(up);
            skinningSurface.SetControlPoint(j, up);
        }

        //Set surface degree
        skinningSurface.SetUDegree(uOrder - 1);
        skinningSurface.SetVDegree(vOrder - 1);

        //Set surface knot
        double_array uKnot = sectionCurve[0].GetKnotArray();
        double_array vKnot = new double_array();

        double_array vChord[] = new double_array[NuControl];
        for (int i = 0; i < NuControl; i++)
        {
            vChord[i] = new double_array();
            vChord[i].Add(0.0);
            for (int j = 0; j < sectionCurve.length - 1; j++)
            {
                math_vector3d v1 = sectionCurve[j].GetControlPoint(i).GetCoord();
                math_vector3d v2 = sectionCurve[j + 1].GetControlPoint(i).GetCoord();
                double chordLength = v1.Distance(v2);
                vChord[i].Add(chordLength);
            }
        }
        double chordSum = 0.0;
        for (int i = 0; i < sectionCurve.length; i++)
        {
            double chordLength = 0.0;
            for (int j = 0; j < NuControl; j++)
            {
                chordLength = chordLength + vChord[j].Get(i);
            }
            chordSum = chordSum + chordLength / (double) vChord.length;
            if (i == 0 || i == sectionCurve.length - 1)
            {
                for (int j = 0; j < vOrder; j++)
                {
                    vKnot.Add(chordSum);
                }
            }
            else
            {
                vKnot.Add(chordSum);
            }
        }
        skinningSurface.SetUknotArray(uKnot);
        skinningSurface.SetVknotArray(vKnot);
        skinningSurface.ToKnotMultiplicity();

        //Set surface border upper and lower limits
        skinningSurface.SetUbound(0, uKnot.Get(0));
        skinningSurface.SetUbound(1, uKnot.GetLast());
        skinningSurface.SetVbound(0, vKnot.Get(0));
        skinningSurface.SetVbound(1, vKnot.GetLast());
    }

    public void SurfaceSkinningWithSpine()
    {
        //Pre-treatment
        skinningSurface = new step_b_spline_surface();
        spineDerivativeVector = new math_vector3d[sectionCurve.length];
        spineProjectPoint = new math_vector3d[sectionCurve.length];
        PreSetSectionCurve();

        //Define basic information
        int NuControl = sectionCurve[0].GetControlPoint().Count();

        //Get spineCurve Derivative Vector Data
        this.GetSpineCurveData();

        //Create linkCurve
        math_vector3d startDeriVector;
        math_vector3d endDeriVector;
        math_vector3d startPoint;
        math_vector3d endPoint;
        double sectionParameter;

        step_b_spline_curve[][] linkCurveSegments = new step_b_spline_curve[NuControl][sectionCurve.length - 1];
        step_b_spline_curve[] linkCurve = new step_b_spline_curve[NuControl];

//        math_vector3d temp = new math_vector3d();
//        temp = spineProjectPoint[1];
//        spineProjectPoint[1] = spineProjectPoint[2];
//        spineProjectPoint[2] = temp;
//
//        temp = spineDerivativeVector[1];
//        spineDerivativeVector[1] = spineDerivativeVector[2];
//        spineDerivativeVector[2] = temp;

        for (int i = 0; i < NuControl; i++)
        {
            for (int j = 0; j < sectionCurve.length - 1; j++)
            {
                startDeriVector = new math_vector3d();
                endDeriVector = new math_vector3d();
                startPoint = new math_vector3d();
                endPoint = new math_vector3d();
                sectionParameter = 0.0;

                sectionParameter = i * (1.0 / (NuControl - 1));
                startDeriVector = this.GetDerivativeVector(i, j);
                endDeriVector = this.GetDerivativeVector(i, j + 1);
                sectionCurve[j].Coordinate(sectionParameter, startPoint);
                sectionCurve[j + 1].Coordinate(sectionParameter, endPoint);
                linkCurveSegments[i][j] = this.FergusonCurve(startPoint, endPoint, startDeriVector, endDeriVector);
            }
        }

        step_curve_array linkCurveSegmentArray;
        step_curve_array joinSegment;
        for (int i = 0; i < NuControl; i++)
        {
            linkCurveSegmentArray = new step_curve_array();
            joinSegment = new step_curve_array();
            for (int j = 0; j < sectionCurve.length - 1; j++)
            {
                linkCurveSegmentArray.Add(linkCurveSegments[i][j]);
            }
            linkCurve[i] = (step_b_spline_curve) linkCurveSegmentArray.Join(0.1, joinSegment);
        }

        step_curve_array linkCurveArray = new step_curve_array();
        for (int i = 0; i < linkCurve.length; i++)
        {
            linkCurveArray.Add(linkCurve[i]);
        }
        //linkCurveArray.WriteToSurfaceModel("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestLink003.stp");

        step_curve_array sectionCurveArray = new step_curve_array();
        for (int i = 0; i < sectionCurve.length; i++)
        {
            sectionCurveArray.Add(sectionCurve[i]);
        }

        prim_curve_network_test_1 skinningSurfaceTest = new prim_curve_network_test_1(false, linkCurveArray, sectionCurveArray);
        skinningSurface = skinningSurfaceTest.GetSumSurface();

    }

    public void SurfaceSkinningWithSpineAndGuide()
    {
    }

    private void PreSetSectionCurve()
    {
        int sectionNum = this.sectionCurve.length;
        //Remove repeat knot in section curve
        for (int i = 0; i < sectionNum; i++)
        {
            short order = this.sectionCurve[i].Order();
            int totalKnot = this.sectionCurve[i].GetKnotMultiplicity().Count();
            int multiplicity = 0;
            double_array removeItem = new double_array();
            for (int j = 0; j < totalKnot; j++)
            {
                multiplicity = this.sectionCurve[i].GetKnotMultiplicity().Get(j);
                if (multiplicity != order && multiplicity != 1)
                {
                    double u = this.sectionCurve[i].GetKnotMultipleArray().Get(j);
                    for (int k = 1; k < multiplicity; k++)
                    {
                        removeItem.Add(u);
                    }
                }
            }
            double uRemove[] = removeItem.ToDouble();
            this.sectionCurve[i].InsertAndRemoveKnots(null, uRemove);
        }

//        //Adjust section curve order
//        for (int i = 0; i < sectionNum; i++)
//        {
//            this.sectionCurve[i] = this.sectionCurve[i].DegreeReductionOrElevation(6);
//            this.sectionCurve[i].ToKnotMultiplicity();
//        }

        //Adjust section curve direction
        math_vector3d mainTangent = new math_vector3d();
        this.sectionCurve[0].Tangent(0.0, mainTangent);
        for (int i = 1; i < sectionNum; i++)
        {
            math_vector3d compareTangent = new math_vector3d();
            this.sectionCurve[i].Tangent(0.0, compareTangent);
            if (mainTangent.DotProduct(compareTangent) <= 0)
            {
                this.sectionCurve[i] = (step_b_spline_curve) this.sectionCurve[i].Reverse();
            }
        }

        //Knot Insertion
        for (int i = 0; i < sectionCurve.length; i++)
        {
            sectionCurve[i].NormalizeKnotValue();
        }

        double_array uArray = new double_array();
        uArray.Add(0.0);
        int counter[] = new int[sectionCurve.length];
        for (int i = 0; i < sectionCurve.length; i++)
        {
            counter[i] = 0;
        }

        boolean isDone;
        do
        {
            isDone = true;
            double u = 0.0;
            int index = 0;

            u = sectionCurve[0].GetKnotMultipleArray().Get(counter[0]);
            index = 0;
            for (int i = 0; i < sectionCurve.length - 1; i++)
            {
                if (u >= sectionCurve[i + 1].GetKnotMultipleArray().Get(counter[i + 1]))
                {
                    u = sectionCurve[i + 1].GetKnotMultipleArray().Get(counter[i + 1]);
                    index = i + 1;
                }
            }
            if (counter[index] < sectionCurve[index].GetKnotMultiplicity().Count() - 1)
            {
                counter[index]++;
            }
            if (u > uArray.GetLast())
            {
                uArray.Add(u);
            }
            for (int i = 0; i < sectionCurve.length; i++)
            {
                isDone = isDone
                        && (counter[i] == sectionCurve[i].GetKnotMultiplicity().Count() - 1);
            }
        }
        while (!isDone);
        uArray.Add(1.0);
        double_array uTemp = new double_array();
        uTemp.Add(0.0);
        for (int i = 0; i < uArray.Size() - 1; i++)
        {
            for (int j = i + 1; j < uArray.Size(); j++)
            //Remove close knot
            {
                if (uArray.Get(j) - uArray.Get(i) <= 0.00001)
                {
                    i++;
                    continue;
                }
                else
                {
                    uTemp.Add(uArray.Get(j));
                    break;
                }
            }
        }
        uArray = new double_array();
        for (int i = 1; i < uTemp.Size() - 1; i++)
        {
            uArray.Add(uTemp.Get(i));
        }

        for (int i = 0; i < sectionCurve.length; i++)
        {
            double_array uRemove = new double_array();
            for (int j = 1; j < sectionCurve[i].GetKnotMultiplicity().Count() - 1; j++)
            {
                uRemove.Add(sectionCurve[i].GetKnotMultipleArray().Get(j));
            }
            sectionCurve[i].InsertAndRemoveKnots(uArray.ToDouble(), uRemove.ToDouble());
        }
    }

    public void GetSpineCurveData()
    {
        double[] doubles;
        math_vector3d targetPointTemp;
        math_vector3d projectPointTemp;
        math_vector3d derivativeVectorTemp;

        for (int i = 0; i < sectionCurve.length; i++)
        {
            targetPointTemp = new math_vector3d();
            projectPointTemp = new math_vector3d();
            derivativeVectorTemp = new math_vector3d();
            doubles = new double[1];

            sectionCurve[i].Coordinate(0.5, targetPointTemp);
            //spineCurve.ProjectOnCurve(targetPointTemp, doubles, projectPointTemp);
            spineCurve.NearestProjectPoint(targetPointTemp, projectPointTemp);
            spineProjectPoint[i] = projectPointTemp;
            spineCurve.Tangent(spineProjectPoint[i], derivativeVectorTemp);
            spineDerivativeVector[i] = derivativeVectorTemp;
        }//Get position & devivative vector of point on spine curve intersect with section curves.

    }

    public math_vector3d GetDerivativeVector(int controlPointNo, int sectionCurveNo)
    {
        double coefficient = 0.0;
        double totalProjectPointDistance = 0.0;
        double totalControlPointDistance = 0.0;
        math_vector3d frontControlPoint = new math_vector3d();
        math_vector3d midControlPoint = new math_vector3d();
        math_vector3d rearControlPoint = new math_vector3d();
        math_vector3d derivativeVector = new math_vector3d();
        if (sectionCurveNo == 0)
        {
            totalProjectPointDistance = spineProjectPoint[sectionCurveNo + 1].Subtract(spineProjectPoint[sectionCurveNo]).Length();
            frontControlPoint = sectionCurve[sectionCurveNo + 1].GetControlPoint(controlPointNo).GetCoord();
            midControlPoint = sectionCurve[sectionCurveNo].GetControlPoint(controlPointNo).GetCoord();
            totalControlPointDistance = frontControlPoint.Subtract(midControlPoint).Length();
            coefficient =/* totalControlPointDistance / */ totalProjectPointDistance;
        }
        else if (sectionCurveNo == sectionCurve.length - 1)
        {
            totalProjectPointDistance = spineProjectPoint[sectionCurveNo].Subtract(spineProjectPoint[sectionCurveNo - 1]).Length();
            midControlPoint = sectionCurve[sectionCurveNo].GetControlPoint(controlPointNo).GetCoord();
            rearControlPoint = sectionCurve[sectionCurveNo - 1].GetControlPoint(controlPointNo).GetCoord();
            totalControlPointDistance = midControlPoint.Subtract(rearControlPoint).Length();
            coefficient =/* totalControlPointDistance / */ totalProjectPointDistance;
        }
        else if (sectionCurveNo > 0 && sectionCurveNo < sectionCurve.length - 1)
        {
            totalProjectPointDistance = spineProjectPoint[sectionCurveNo + 1].Subtract(spineProjectPoint[sectionCurveNo]).Length()
                    + spineProjectPoint[sectionCurveNo].Subtract(spineProjectPoint[sectionCurveNo - 1]).Length();
            frontControlPoint = sectionCurve[sectionCurveNo + 1].GetControlPoint(controlPointNo).GetCoord();
            midControlPoint = sectionCurve[sectionCurveNo].GetControlPoint(controlPointNo).GetCoord();
            rearControlPoint = sectionCurve[sectionCurveNo - 1].GetControlPoint(controlPointNo).GetCoord();
            totalControlPointDistance = frontControlPoint.Subtract(midControlPoint).Length()
                    + midControlPoint.Subtract(rearControlPoint).Length();
            coefficient =/* totalControlPointDistance / */ totalProjectPointDistance;
        }
        derivativeVector = spineDerivativeVector[sectionCurveNo].Multiply(coefficient);
        return derivativeVector;
        //Get derivative vector of target control point on target section point.
    }

    public step_b_spline_curve FergusonCurve(math_vector3d dataPoint1, math_vector3d dataPoint2, math_vector3d deriVector1, math_vector3d deriVector2)
    {
        math_vector3d[] furgusonData =
        {
            dataPoint1, dataPoint2, deriVector1, deriVector2
        };
        double[][] fergusonCoefficient =
        {
            {
                1, 0, 0, 0
            },
            {
                0, 0, 1, 0
            },
            {
                -3, 3, -2, -1
            },
            {
                2, -2, 1, 1
            }
        };
        double[][] bezierCoefficient =
        {
            {
                1, 0, 0, 0
            },
            {
                -3, 3, 0, 0
            },
            {
                3, -6, 3, 0
            },
            {
                -1, 3, -3, 1
            }
        };
        math_matrix fergusonMatrix = new math_matrix(fergusonCoefficient);
        math_matrix bezierMatrix = new math_matrix(bezierCoefficient);
        math_matrix bezierMatrixInverse = bezierMatrix.Inverse();
        math_matrix finalMatrix = bezierMatrixInverse.Multiply(fergusonMatrix);
        math_vector3d[] furgusonControlPoint = new math_vector3d[4];
        for (int i = 0; i < 4; i++)
        {
            furgusonControlPoint[i] = furgusonData[0].Multiply(finalMatrix.GetElement(i, 0)).Add(furgusonData[1].Multiply(finalMatrix.GetElement(i, 1))).Add(furgusonData[2].Multiply(finalMatrix.GetElement(i, 2))).Add(furgusonData[3].Multiply(finalMatrix.GetElement(i, 3)));
        }
        step_b_spline_curve furgusonCurve = new step_b_spline_curve(4, furgusonControlPoint);

        return furgusonCurve;
    }
}
