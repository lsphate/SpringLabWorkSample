package SkinningSurface;

import java.util.ArrayList;
import java.util.Collections;
import SpringSolid.Mathematics.*;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;
import SpringSolid.MinimumDistance.curve_curve_distance;
import java.util.Random;

public class prim_curve_network_test_1 extends step_open_shell
{
    private step_curve_array guideCurveArray;
    private step_curve_array profileCurveArray;
    private ArrayList<math_vector3d_array> projectPointArray;
    private step_b_spline_surface guideSurface;
    private step_b_spline_surface profileSurface;
    private step_b_spline_surface tensorProductSurface;
    private step_b_spline_surface sumSurface;
    private String localPath;
    private step_curve_array testStepCurveArray;//Leon

    public step_b_spline_surface GetSumSurface()
    {
        return this.sumSurface;
    }  /* end of procedure */


    public prim_curve_network_test_1()
    {
        this.guideCurveArray = null;
        this.profileCurveArray = null;
        this.projectPointArray = new ArrayList<math_vector3d_array>();
    }

    public prim_curve_network_test_1(step_curve_array sCurveArray) //by Leon
    {
        this.testStepCurveArray = sCurveArray;
    }

    public prim_curve_network_test_1(boolean net, step_curve_array guide, step_curve_array profile)
    {
        this.localPath = System.getProperty("user.dir");
        this.localPath = this.localPath.concat("\\surfaceFile\\");

        this.projectPointArray = new ArrayList<math_vector3d_array>();

        if (net == true)
        {
            this.CalculateMinimumDistance(guide, profile);

            guide = this.CreateNewGuideCurve(guide);
            profile = this.CreateNewProfileCurve(profile);
        }

        this.guideCurveArray = guide;
        this.profileCurveArray = profile;

        // guide.WriteToSurfaceModel("Test11.stp");
        // profile.WriteToSurfaceModel("Test22.stp");

        int nguide = this.guideCurveArray.Count();
        step_b_spline_curve bsp;
        for (int c1 = 0; c1 < nguide; c1++)
        {
            bsp = (step_b_spline_curve) this.guideCurveArray.Get(c1);
            bsp.SetExtremeFlag(false);
            bsp.FindBreakPoint();
            bsp.DataPointFromControlPoint();
        }

        int nprofile = this.profileCurveArray.Count();
        for (int c1 = 0; c1 < nprofile; c1++)
        {
            bsp = (step_b_spline_curve) this.profileCurveArray.Get(c1);
            bsp.SetExtremeFlag(false);
            bsp.FindBreakPoint();
            bsp.DataPointFromControlPoint();
        }

        this.CreateSkinnedSurface(this.guideCurveArray, 0);
        this.CreateSkinnedSurface(this.profileCurveArray, 1);
        this.CreateTensorProduct();

        this.CreateSumSurface();

        //sumSurface.WriteToSurfaceModel("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSurface_leon_03.stp");
        // this.UpdateSumSurface();

//        this.CreateRelatedTopology();
//
//        this.outputSTP();
    }  /* end of procedure prim_curve_network */


    public prim_curve_network_test_1(step_curve_array guide, step_curve_array profile, step_surface supnt)
    {
        this.guideCurveArray = guide;
        this.profileCurveArray = profile;
        this.sumSurface = (step_b_spline_surface) supnt;

        this.UpdateSumSurface();

        this.CreateRelatedTopology();

        this.outputSTP();
    }  /* end of procedure prim_curve_network */


    public void CalculateMinimumDistance(step_curve_array guideCurveArray, step_curve_array profileCurveArray)
    {
        long time = -System.currentTimeMillis();

        math_vector3d coord1;
        int nprofile = profileCurveArray.Size();
        int nguide = guideCurveArray.Size();
        for (int c1 = 0; c1 < nprofile; c1++)
        {
            step_curve profile = profileCurveArray.Get(c1);
            math_vector3d_array ptArray = new math_vector3d_array();

            for (int c2 = 0; c2 < nguide; c2++)
            {
                step_curve guide = guideCurveArray.Get(c2);

                curve_curve_distance distance = new curve_curve_distance();

                distance.SetTwoCurves(guide, profile);
                distance.FindFeasibleDirection();
                coord1 = distance.GetProjectCoord1();

                ptArray.Add(coord1);
            }

            this.projectPointArray.add(ptArray);
        }
        System.out.println("尋找最短距離點時間: " + (time + System.currentTimeMillis()) / 1000f);
    }

    private step_curve_array CreateNewGuideCurve(step_curve_array guideCurveArray)
    {
        math_vector3d_array startProfile = this.projectPointArray.get(0);
        math_vector3d_array endProfile = this.projectPointArray.get(this.projectPointArray.size() - 1);
        step_curve_array newGuideCurveArray = new step_curve_array();

        for (int c1 = 0; c1 < guideCurveArray.Count(); c1++)
        {
            step_curve guideCurve = guideCurveArray.Get(c1);
            math_vector3d startVertex = startProfile.Get(c1);
            math_vector3d endVertex = endProfile.Get(c1);

            //判斷方向
            math_vector3d u0pt = guideCurve.StartCoord();
            math_vector3d u1pt = guideCurve.EndCoord();

            if (u0pt.Distance(startVertex) > u1pt.Distance(startVertex))
            {
                guideCurve = guideCurve.Reverse();
            }

            //
            step_curve curve1 = guideCurve.NewSubdivision(startVertex, endVertex);
            if (curve1.IsBsplineCurve() == true)
            {
                step_b_spline_curve bsp = (step_b_spline_curve) curve1;
                bsp.NormalizeKnotValue();
            }
            else
            {
                curve1 = curve1.ToBsplineCurve();
                step_b_spline_curve bsp = (step_b_spline_curve) curve1;

                bsp.NormalizeKnotValue();
            }

            newGuideCurveArray.Add(curve1);
        }
        return newGuideCurveArray;
    }

    private step_curve_array CreateNewProfileCurve(step_curve_array profileCurveArray)
    {
        math_vector3d_array ptArray = null;
        int num = profileCurveArray.Count();
        step_curve_array newProfileCurveArray = new step_curve_array();
        for (int c1 = 0; c1 < num; c1++)
        {
            step_curve profileCurve = profileCurveArray.Get(c1);

            ptArray = this.projectPointArray.get(c1);

            if (ptArray.Count() == 2)
            {
                math_vector3d startVertex = ptArray.Get(0);
                math_vector3d endVertex = ptArray.Get(1);

                //判斷方向
                math_vector3d u0pt = profileCurve.StartCoord();
                math_vector3d u1pt = profileCurve.EndCoord();
                double dist1 = u0pt.Distance(startVertex);
                double dist2 = u1pt.Distance(startVertex);
                if (dist1 > dist2)
                {
                    profileCurve = profileCurve.Reverse();
                }

                step_b_spline_curve curve1 = (step_b_spline_curve) profileCurve.NewSubdivision(startVertex, endVertex);
                curve1.NormalizeKnotValue();

                newProfileCurveArray.Add(curve1);
            }
            else
            {
                math_vector3d extrem[] = new math_vector3d[2];

                profileCurve.FindExtremeRemotePoints(ptArray, extrem);

                math_vector3d startVertex = extrem[0];
                math_vector3d endVertex = extrem[1];

                //判斷方向
                math_vector3d u0pt = profileCurve.StartCoord();
                math_vector3d u1pt = profileCurve.EndCoord();
                double dist1 = u0pt.Distance(startVertex);
                double dist2 = u1pt.Distance(startVertex);
                if (dist1 > dist2)
                {
                    profileCurve = profileCurve.Reverse();
                }

                step_curve curve1 = profileCurve.NewSubdivision(startVertex, endVertex);
                if (curve1.IsBsplineCurve() == true)
                {
                    step_b_spline_curve bsp = (step_b_spline_curve) curve1;

                    bsp.NormalizeKnotValue();
                }
                else
                {
                    curve1 = curve1.ToBsplineCurve();
                    step_b_spline_curve bsp = (step_b_spline_curve) curve1;

                    bsp.NormalizeKnotValue();
                }
                newProfileCurveArray.Add(curve1);
            }
        }
        return newProfileCurveArray;
    }

    public step_curve_array KnotRemoveAndInsert(step_curve_array curveArray)
    {
        ArrayList<Double> knotArray = new ArrayList<Double>();
        double_array knotVector;

        for (int c1 = 0; c1 < curveArray.Size(); c1++)
        {
            step_b_spline_curve curve = (step_b_spline_curve) curveArray.Get(c1);

            int order = curve.Order();

            for (int c2 = order; c2 < curve.KnotCount() - order; c2++)
            {
                knotArray.add(curve.GetKnot(c2));
            }
        }

        Collections.sort(knotArray);

        //統整global knot vector
        for (int c1 = 0; c1 < knotArray.size() - 1; c1++)
        {
            double d1 = knotArray.get(c1);
            double d2 = knotArray.get(c1 + 1);

            if (Math.abs(d2 - d1) <= 0.001)
            {
                knotArray.remove(c1 + 1);
                c1--;
            }
        }

        knotVector = new double_array();
        double_array addKnotArray;
        double_array removeKnotArray;
        for (int c1 = 0; c1 < knotArray.size(); c1++)
        {
            knotVector.Add(knotArray.get(c1));
        }

        for (int c1 = 0; c1 < curveArray.Size(); c1++)
        {
            step_b_spline_curve curve = (step_b_spline_curve) curveArray.Get(c1);
            addKnotArray = knotVector.Copy();
            removeKnotArray = new double_array();

            boolean _bool;
            int order = curve.Order();

            // remove knot
            for (int c2 = order; c2 < curve.KnotCount() - order; c2++)
            {
                double u = curve.GetKnot(c2);
                _bool = true;

                for (int c3 = 0; c3 < addKnotArray.Size(); c3++)
                {
                    if (u == addKnotArray.Get(c3))
                    {
                        _bool = false;
                        break;
                    }
                }

                if (_bool)
                {
                    removeKnotArray.Add(u);
                }
            }

            // add knot
            for (int c2 = 0; c2 < addKnotArray.Size(); c2++)
            {
                double u = addKnotArray.Get(c2);

                for (int c3 = order; c3 < curve.KnotCount() - order; c3++)
                {
                    if (u == curve.GetKnot(c3))
                    {
                        addKnotArray.Delete(c2);
                        c2--;
                        break;
                    }
                }
            }

            curve.InsertAndRemoveKnots(addKnotArray.ToDouble(), removeKnotArray.ToDouble());
        }

        return curveArray;
    }

    public void CreateSkinnedSurface(step_curve_array curveArray, int type) //0:guide , 1:profile
    {
        //curveArray.WriteToSurfaceModel("OriginalCurve02.stp");

        step_b_spline_curve curve1 = (step_b_spline_curve) curveArray.Get(0);
        double_array cknotVector = curve1.GetKnotArray();
        math_vector3d_array dataArray[] = new math_vector3d_array[curve1.GetControlPoint().Size()];

        double newKnotValue[] = new double[curveArray.Size() - 1];
        double_array newKnotArray = new double_array();
        step_b_spline_curve bsp = null;
        math_vector3d_array ptArray;
        int newCurveOrder = 0;

        step_curve_array crossCurveArray = new step_curve_array();

        //設定V方向curve
        for (int c1 = 0; c1 < curve1.GetControlPoint().Size(); c1++)
        {
            ptArray = new math_vector3d_array();

            for (int c2 = 0; c2 < curveArray.Size(); c2++)
            {
                step_b_spline_curve curve = (step_b_spline_curve) curveArray.Get(c2);
                ptArray.Add(curve.GetControlPoint(c1).GetCoord());

                if (c2 >= 1)
                {
                    math_vector3d pt1 = ptArray.Get(c2);
                    math_vector3d pt2 = ptArray.Get(c2 - 1);
                    newKnotValue[c2 - 1] = newKnotValue[c2 - 1] + pt1.Distance(pt2);

                    if (c1 == curve1.GetControlPoint().Size() - 1)
                    {
                        newKnotValue[c2 - 1] = newKnotValue[c2 - 1] / curve1.GetControlPoint().Size();
                    }
                }
            }

            dataArray[c1] = ptArray;
            bsp = new step_b_spline_curve();
            bsp.Interpolation(ptArray);
            bsp.NormalizeKnotValue();
            crossCurveArray.Add(bsp);
            newCurveOrder = bsp.Order();
        }

        bsp = (step_b_spline_curve) crossCurveArray.Get(0);

        //設定另一方向knot vector
        for (int c1 = 1; c1 < newKnotValue.length; c1++)
        {
            newKnotValue[c1] = newKnotValue[c1] + newKnotValue[c1 - 1];
        }

        for (int c1 = 0; c1 < curveArray.Size(); c1++)
        {
            if (c1 == 0)
            {
                for (int c2 = 0; c2 < newCurveOrder; c2++)
                {
                    newKnotArray.Add(0.0);
                }
            }
            else if (c1 == curveArray.Size() - 1)
            {
                for (int c2 = 0; c2 < newCurveOrder; c2++)
                {
                    newKnotArray.Add(1.0);
                }
            }
            else
            {
                newKnotArray.Add((newKnotValue[c1 - 1] / newKnotValue[newKnotValue.length - 1]));
            }
        }

        //crossCurveArray.WriteToSurfaceModel("CrossCurve02.stp");

        // 開始設定skinnedSurface
        math_upoint uArray;
        step_b_spline_surface skinnedSurface = new step_b_spline_surface();

        if (type == 0) //guide
        {
            //設定控制點
            skinnedSurface.SetNucontrol(curve1.Ncontrol());
            skinnedSurface.SetNvcontrol(bsp.Ncontrol());
            step_b_spline_curve bspcurve = (step_b_spline_curve) crossCurveArray.Get(0);
            int controlPointNumber = bspcurve.Ncontrol();
            for (int c1 = 0; c1 < controlPointNumber; c1++)
            {
                uArray = new math_upoint();
                for (int c2 = 0; c2 < crossCurveArray.Count(); c2++)
                {
                    step_b_spline_curve curve = (step_b_spline_curve) crossCurveArray.Get(c2);
                    uArray.AddControlPoint(curve.GetControlPoint(c1));
                }
                skinnedSurface.SetControlPoint(c1, uArray);
            }

            //設定Degree
            skinnedSurface.SetUDegree(curve1.Order() - 1);
            skinnedSurface.SetVDegree(newCurveOrder - 1);
            //設定Knot Vector

            skinnedSurface.SetUknotArray(cknotVector);
            skinnedSurface.SetVknotArray(newKnotArray);
            skinnedSurface.ToKnotMultiplicity();

            //設定U V Bound
            skinnedSurface.SetUbound(0, cknotVector.Get(0));
            skinnedSurface.SetUbound(1, cknotVector.GetLast());
            skinnedSurface.SetVbound(0, newKnotArray.Get(0));
            skinnedSurface.SetVbound(1, newKnotArray.GetLast());
            skinnedSurface.ToKnotMultiplicity();
            skinnedSurface.SetWeightData();

            //skinnedSurface.WriteToSurfaceModel("SkinSurface01.stp");

            this.guideSurface = skinnedSurface;
        }
        else if (type == 1) //profile
        {
            //設定控制點
            skinnedSurface.SetNucontrol(bsp.Ncontrol());
            skinnedSurface.SetNvcontrol(curve1.Ncontrol());
            for (int c1 = 0; c1 < crossCurveArray.Count(); c1++)
            {
                step_b_spline_curve curve = (step_b_spline_curve) crossCurveArray.Get(c1);
                uArray = new math_upoint();

                for (int c2 = 0; c2 < curve.GetControlPoint().Size(); c2++)
                {
                    uArray.AddControlPoint(curve.GetControlPoint(c2));
                }

                skinnedSurface.SetControlPoint(c1, uArray);
            }

            //設定Degree
            skinnedSurface.SetUDegree(newCurveOrder - 1);
            skinnedSurface.SetVDegree(curve1.Order() - 1);

            //設定Knot Vector
            skinnedSurface.SetUknotArray(newKnotArray);
            skinnedSurface.SetVknotArray(cknotVector);
            skinnedSurface.ToKnotMultiplicity();

            //設定U V Bound
            skinnedSurface.SetUbound(0, newKnotArray.Get(0));
            skinnedSurface.SetUbound(1, newKnotArray.GetLast());
            skinnedSurface.SetVbound(0, cknotVector.Get(0));
            skinnedSurface.SetVbound(1, cknotVector.GetLast());
            skinnedSurface.ToKnotMultiplicity();
            skinnedSurface.SetWeightData();
            //skinnedSurface.WriteToSurfaceModel("SkinSurface02.stp");
            this.profileSurface = skinnedSurface;
        }
    }  /* end of procedure CreateSkinnedSurface */


    public void CreateTensorProduct()
    {
        int unumber = this.profileCurveArray.Size();
        int vnumber = this.guideCurveArray.Size();

        this.tensorProductSurface = new step_b_spline_surface();

        math_vector3d_array ptArray = new math_vector3d_array();
        math_vector3d_array tempArray = new math_vector3d_array();
        int nguide = this.guideCurveArray.Size();
        int nprofile = this.profileCurveArray.Size();
        math_vector3d common = null;
        for (int c1 = 0; c1 < nguide; c1++)
        {
            step_b_spline_curve guide = (step_b_spline_curve) this.guideCurveArray.Get(c1);
            for (int c2 = 0; c2 < nprofile; c2++)
            {
                step_b_spline_curve profile = (step_b_spline_curve) this.profileCurveArray.Get(c2);
                tempArray = new math_vector3d_array();
                guide.CurveIntersect(profile, tempArray);

                if (tempArray.Count() >= 1)
                {
                    common = tempArray.Get(0);
                    ptArray.Add(common);
                }
                else
                {
                    System.out.println("Create Surface:Found Min Dis Point between Curve in Tensor Product Process");
                    common = this.MinDisTwoCurve(guide, profile);
                    ptArray.Add(common);
                }
            }
        }

//        System.out.println("wow");
//        guideCurveArray.WriteToSurfaceModel("D:\\guideCurveArray.stp");
//        profileCurveArray.WriteToSurfaceModel("D:\\profileCurveArray.stp");
//        ptArray.WriteToSurfaceModel("D:\\ptArray.stp");

        this.tensorProductSurface.Interpolation(unumber, vnumber, ptArray);
        this.tensorProductSurface.NormalizeKnotValues();
        this.tensorProductSurface.ToKnotMultiplicity();
        this.tensorProductSurface.SetWeightData();
    }  /* end of procedure CreateTensorProduct */


    public void CreateSumSurface()
    {
        this.guideSurface.NormalizeKnotValues();
        this.profileSurface.NormalizeKnotValues();
        this.tensorProductSurface.NormalizeKnotValues();

        int UmaxOrder = 0;
        int VmaxOrder = 0;

        double_array finalUvectorArray = new double_array();
        double_array finalVvectorArray = new double_array();
        step_b_spline_surface surfArray[] =
        {
            guideSurface, profileSurface, tensorProductSurface
        };

        //this.guideSurface.WriteToSurfaceModel("Guide.stp");
        //this.profileSurface.WriteToSurfaceModel("profile.stp");
        //this.tensorProductSurface.WriteToSurfaceModel("tensor.stp");

        // elevate to the same degree
        // 升至一樣階數
        int u, v;

        // choice the maximum value of U and V
        // 取得最大U V
        for (int c1 = 0; c1 < 3; c1++)
        {
            u = surfArray[c1].Uorder();
            v = surfArray[c1].Vorder();
            if (u > UmaxOrder)
            {
                UmaxOrder = u;
            }
            if (v > VmaxOrder)
            {
                VmaxOrder = v;
            }
        }

        for (int c1 = 0; c1 < 3; c1++)
        {
            u = surfArray[c1].Uorder();
            v = surfArray[c1].Vorder();
            if (u < UmaxOrder || v < VmaxOrder)
            {
                step_b_spline_surface bsp;
                bsp = surfArray[c1].DegreeReductionOrElevation(UmaxOrder, VmaxOrder);
                surfArray[c1] = bsp;
                bsp.NormalizeKnotValues();
                if (c1 == 0)
                {
                    this.guideSurface = bsp;
                }
                else if (c1 == 1)
                {
                    this.profileSurface = bsp;
                }
                else if (c1 == 2)
                {
                    this.tensorProductSurface = bsp;
                }
            }
        }

        // 調整三個面之knot vector至一樣
        // adjust the knot vector of three surfaces as the same
//       this.SetFinalKnotVector(0);
//       this.SetFinalKnotVector(1);
        this.SetFinalVector2();

//        this.guideSurface.WriteToSurfaceModel("GuideSurface.stp");
//        this.profileSurface.WriteToSurfaceModel("ProfileSurface.stp");
//        this.tensorProductSurface.WriteToSurfaceModel("TensorProductSurface.stp");

        // 建立曲面
        this.sumSurface = new step_b_spline_surface();

        int nuControl = this.guideSurface.Nucontrol();
        int nvControl = this.guideSurface.Nvcontrol();

        finalUvectorArray = this.guideSurface.GetUknotArray();
        finalVvectorArray = this.guideSurface.GetVknotArray();
        math_upoint_array controlPointArray = this.SetFinalControlPoint();

        // 設定控制點
        this.sumSurface.SetNucontrol(nuControl);
        this.sumSurface.SetNvcontrol(nvControl);

        for (int c1 = 0; c1 < controlPointArray.Size(); c1++)
        {
            this.sumSurface.SetControlPoint(c1, controlPointArray.Get(c1));
        }

        // 設定Degree
        this.sumSurface.SetUDegree(UmaxOrder - 1);
        this.sumSurface.SetVDegree(VmaxOrder - 1);

        // 設定knot
        this.sumSurface.SetUknotArray(finalUvectorArray);
        this.sumSurface.SetVknotArray(finalVvectorArray);
        this.sumSurface.ToKnotMultiplicity();

        //設定上下邊界
        this.sumSurface.SetUbound(0, finalUvectorArray.Get(0));
        this.sumSurface.SetUbound(1, finalUvectorArray.GetLast());
        this.sumSurface.SetVbound(0, finalVvectorArray.Get(0));
        this.sumSurface.SetVbound(1, finalVvectorArray.GetLast());
        this.sumSurface.ToKnotMultiplicity();
        this.sumSurface.SetWeightData();

        //this.sumSurface.WriteToSurfaceModel("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSurface_leon_03.stp");
    }  /* end of procedure CreateSumlSurface */


    public void outputSTP()
    {
        String path1 = this.localPath.concat("guideSurface.stp");
        String path2 = this.localPath.concat("profileSurface.stp");
        String path3 = this.localPath.concat("tensorProductSurface.stp");
        String path4 = this.localPath.concat("finalSurface.stp");
        String path5 = this.localPath.concat("finalCurve.stp");

        //this.guideSurface.WriteToSurfaceModel(path1);
        //this.profileSurface.WriteToSurfaceModel(path2);
        //this.tensorProductSurface.WriteToSurfaceModel(path3);
        //this.sumSurface.WriteToSurfaceModel(path4);

        step_curve_array totalCurveArray = new step_curve_array();
        for (int c1 = 0; c1 < this.guideCurveArray.Size(); c1++)
        {
            totalCurveArray.Add(this.guideCurveArray.Get(c1));
        }

        for (int c1 = 0; c1 < this.profileCurveArray.Size(); c1++)
        {
            totalCurveArray.Add(this.profileCurveArray.Get(c1));
        }

        //totalCurveArray.WriteToSurfaceModel(path5);
    }

    public math_upoint_array SetFinalControlPoint()
    {
        // this.guideSurface.WriteToSurfaceModel("Guide1.stp");
        // this.profileSurface.WriteToSurfaceModel("profile1.stp");
        // this.tensorProductSurface.WriteToSurfaceModel("tensor1.stp");

        math_upoint newUpointArray;
        math_vector3d pt1;
        math_vector3d pt2;
        math_vector3d pt3;
        math_vector3d tempPT;
        step_cartesian_point finalPoint;
        math_upoint upointArray = new math_upoint();
        math_upoint_array controlPointArray = new math_upoint_array();

        for (int c1 = 0; c1 < this.guideSurface.GetControlPoint().Size(); c1++)
        {
            upointArray = this.guideSurface.GetControlPoint().Get(c1);
            newUpointArray = new math_upoint();

            for (int c2 = 0; c2 < upointArray.GetPoint().Size(); c2++)
            {
                pt1 = this.guideSurface.GetControlPoint().Get(c1).GetPoint().Get(c2).GetCoord();
                pt2 = this.profileSurface.GetControlPoint().Get(c1).GetPoint().Get(c2).GetCoord();
                pt3 = this.tensorProductSurface.GetControlPoint().Get(c1).GetPoint().Get(c2).GetCoord();

                tempPT = pt1.Add(pt2).Subtract(pt3);
                finalPoint = new step_cartesian_point(tempPT.X(), tempPT.Y(), tempPT.Z());
                newUpointArray.AddControlPoint(finalPoint);
            }
            controlPointArray.Add(newUpointArray);
        }

        return controlPointArray;
    }  /* end of procedure SetFinalControl */


    public void SetFinalKnotVector(int type) //type 0:U 1:V
    {
        step_b_spline_surface surArray[] =
        {
            this.guideSurface, this.profileSurface, this.tensorProductSurface
        };

        ArrayList<Double> knotArray = new ArrayList<Double>();
        double_array knotVector = new double_array();
        double_array insertArray;
        double_array removeArray;

        if (type == 0) //U
        {
            for (int c1 = 0; c1 < 3; c1++)
            {
                step_b_spline_surface stb = surArray[c1];
                int Uorder = stb.Uorder();
                double s;
                for (int c2 = Uorder - 1; c2 < stb.UknotCount() - Uorder + 1; c2++)
                {
                    s = stb.GetUknotArray().Get(c2);
                    knotArray.add(s);
                }
            }

            Collections.sort(knotArray);
            for (int c1 = 0; c1 < knotArray.size() - 1; c1++)
            {
                double d1 = knotArray.get(c1);
                double d2 = knotArray.get(c1 + 1);

                if (Math.abs(d2 - d1) <= 0.01)
                {
                    knotArray.remove(c1 + 1);
                    c1--;
                }
            }

            //所有的KNOT VALUE
            for (int c1 = 0; c1 < knotArray.size(); c1++)
            {
                knotVector.Add(knotArray.get(c1));
            }

            for (int c1 = 0; c1 < 3; c1++)
            {
                step_b_spline_surface sub = surArray[c1];
                insertArray = knotVector.Copy();
                removeArray = new double_array();

                boolean answer;
                int order = sub.Uorder();

                //remove knot
                for (int c2 = order; c2 < sub.UknotCount() - order; c2++)
                {
                    double u = sub.GetUknotArray().Get(c2);
                    answer = true;
                    for (int c3 = 0; c3 < insertArray.Size(); c3++)
                    {
                        if (u == insertArray.Get(c3))
                        {
                            answer = false;
                            break;
                        }
                    }
                    if (answer)
                    {
                        removeArray.Add(u);
                    }
                }

                // add knot
                for (int c2 = 0; c2 < insertArray.Size(); c2++)
                {
                    double u = insertArray.Get(c2);

                    for (int c3 = order - 1; c3 < sub.UknotCount() - order + 1; c3++)
                    {
                        if (u == sub.GetUknotArray().Get(c3))
                        {
                            insertArray.Delete(c2);
                            c2--;
                            break;
                        }
                    }
                }

                sub.InsertAndRemoveKnots(insertArray.ToDouble(), null, removeArray.ToDouble(), null);
            }
        }
        else //V
        {
            for (int c1 = 0; c1 < 3; c1++)
            {
                step_b_spline_surface stb = surArray[c1];
                int Vorder = stb.Vorder();
                for (int c2 = Vorder - 1; c2 < stb.VknotCount() - Vorder + 1; c2++)
                {
                    knotArray.add(stb.GetVknotArray().Get(c2));
                }
            }

            Collections.sort(knotArray);
            for (int c1 = 0; c1 < knotArray.size() - 1; c1++)
            {
                double d1 = knotArray.get(c1);
                double d2 = knotArray.get(c1 + 1);

                if (Math.abs(d2 - d1) <= 0.01)
                {
                    knotArray.remove(c1 + 1);
                    c1--;
                }
            }

            //所有的KNOT VALUE
            for (int c1 = 0; c1 < knotArray.size(); c1++)
            {
                knotVector.Add(knotArray.get(c1));
            }

            for (int c1 = 0; c1 < 3; c1++)
            {
                step_b_spline_surface sub = surArray[c1];
                insertArray = knotVector.Copy();
                removeArray = new double_array();

                boolean answer;
                int order = sub.Vorder();

                //remove knot
                for (int c2 = order; c2 < sub.VknotCount() - order; c2++)
                {
                    double v = sub.GetVknotArray().Get(c2);
                    answer = true;
                    for (int c3 = 0; c3 < insertArray.Size(); c3++)
                    {
                        if (v == insertArray.Get(c3))
                        {
                            answer = false;
                            break;
                        }
                    }
                    if (answer)
                    {
                        removeArray.Add(v);
                    }
                }

                // add knot
                for (int c2 = 0; c2 < insertArray.Size(); c2++)
                {
                    double v = insertArray.Get(c2);

                    for (int c3 = order - 1; c3 < sub.VknotCount() - order + 1; c3++)
                    {
                        if (v == sub.GetVknotArray().Get(c3))
                        {
                            insertArray.Delete(c2);
                            c2--;
                            break;
                        }
                    }
                }
                sub.InsertAndRemoveKnots(null, insertArray.ToDouble(), null, removeArray.ToDouble());
            }
        }
    }  /* end of procedure SetFinalKnotVector */


    public void CreateRelatedTopology()
    {
        step_advanced_face adv = new step_advanced_face();
        adv.SetFromSurface(this.sumSurface);

        this.LinkAdvancedFace(adv);
    }  /* end of procedure CreateRelatedTopology */


    public static step_curve_array ReadProfiles()
    {
        step_read read1 = new step_read();
        read1.ReadData("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSection03.stp");
        read1.ConversionData(1.0);
        read1.ReEvaluateData();

        step_curve cpnt;
        step_geometric_set gc;
        step_curve_array sumArray = new step_curve_array();
        step_curve_array carray = null;
        step_curve first = null;
        int j = 0;
        int count = read1.shapeModel.geometricSetArray.Count();
        while (j < count)
        {
            gc = read1.shapeModel.geometricSetArray.Get(j);
            carray = gc.GetCurveArray();
            int ncurve = carray.Count();
            for (int icurve = 0; icurve < ncurve; ++icurve)
            {
                cpnt = gc.GetCurveArray().Get(icurve);
                if (cpnt.Type() == COMPOSITE_CURVE)
                {
                    step_composite_curve comp = (step_composite_curve) cpnt;
                    step_curve_array totalArray = new step_curve_array();
                    comp.GetCurves(totalArray);

                    int ntotal = totalArray.Count();
                    for (int k = 0; k < ntotal; ++k)
                    {
                        cpnt = totalArray.Get(k);
                        first = (step_b_spline_curve) cpnt;
                        sumArray.Add(first);
                    }
                }
            }
            ++j;
        }
        return sumArray;
    }

    public static step_curve_array ReadGuides()
    {
        step_read read1 = new step_read();
        read1.ReadData("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestLink03.stp");
        read1.ConversionData(1.0);
        read1.ReEvaluateData();

        long id;
        step_curve cpnt;
        step_geometric_set gc;
        step_curve_array sumArray = new step_curve_array();
        step_curve_array carray = null;
        step_curve first = null;
        int j = 0;
        int count = read1.shapeModel.geometricSetArray.Count();
        while (j < count)
        {
            gc = read1.shapeModel.geometricSetArray.Get(j);
            carray = gc.GetCurveArray();
            int ncurve = carray.Count();
            for (int icurve = 0; icurve < ncurve; ++icurve)
            {
                cpnt = gc.GetCurveArray().Get(icurve);
                if (cpnt.Type() == COMPOSITE_CURVE)
                {
                    step_composite_curve comp = (step_composite_curve) cpnt;
                    step_curve_array totalArray = new step_curve_array();
                    comp.GetCurves(totalArray);

                    int ntotal = totalArray.Count();
                    for (int k = 0; k < ntotal; ++k)
                    {
                        cpnt = totalArray.Get(k);
                        first = (step_b_spline_curve) cpnt;
                        sumArray.Add(first);
                    }
                }
            }
            ++j;
        }

        return sumArray;
    }

    public math_vector3d MinDisTwoCurve(step_curve guide, step_curve profile)// 嚙瞎嚙踝蕭拑u嚙稿嚙踝蕭嚙瘢
    {
        ArrayList<math_vector3d> guidePointArray = new ArrayList<math_vector3d>();
        ArrayList<math_vector3d> profilePointArray = new ArrayList<math_vector3d>();

        int number = 2500;
        double guidePFitness[] = new double[number];
        double profilePFitness[] = new double[number];
        double guidePBestU[] = new double[number];
        double profilePBestU[] = new double[number];
        double guidePCurrentU[] = new double[number];
        double profilePCurrentU[] = new double[number];
        double guideV[] = new double[number];
        double profileV[] = new double[number];
        // 嚙踝蕭嚙談迎蕭N嚙誹群
        for (int c1 = 0; c1 < number; c1++)
        {
            double u = c1 / (Double.parseDouble(String.valueOf(number - 1)));
            double guideBound[] = guide.Bound();//Leon
            double profileBound[] = profile.Bound();//Leon
            double u1 = c1 * (guideBound[1] - guideBound[0]) / number;//Leon 
            double u2 = c1 * (profileBound[1] - profileBound[0]) / number;//Leon         
            math_vector3d guidePoint = new math_vector3d();
            math_vector3d profilePoint = new math_vector3d();
//            guide.Coordinate(u, guidePoint);
            guide.Coordinate(u1, guidePoint);
//            profile.Coordinate(u, profilePoint);
            profile.Coordinate(u2, profilePoint);

            guidePointArray.add(guidePoint);
            profilePointArray.add(profilePoint);
            guidePFitness[c1] = Double.MAX_VALUE;//???
            profilePFitness[c1] = Double.MAX_VALUE;
            guidePBestU[c1] = u;
            profilePBestU[c1] = u;
            guidePCurrentU[c1] = u;
            profilePCurrentU[c1] = u;
            guideV[c1] = 0.0f;
            profileV[c1] = 0.0f;
        }

        double guideGlobalDis = Double.MAX_VALUE;
        double profileGlobalDis = Double.MAX_VALUE;
        double distance = Double.MAX_VALUE;
        double guideGBestU = 0;
        double profileGBestU = 0;
        double w = 1.0f;// 0.729
        double const1 = 1.5f;
        double const2 = 1.5f;
        double vupper = 2.0f;
        double vlower = -2.0f;
        double deltaT = 0.001f;
        Random rand1 = new Random();
        Random rand2 = new Random();
        int randRange = 1000;
        int counter = 0;

        double smallestDis = Double.MAX_VALUE;//Leon
        boolean isNext = true;//Leon

        while (isNext)//counter < 100) // Math.abs( lastSolution - currentSolution) >=// 0.001
        {
            // 嚙瞎嚙緩嚙磊嚙瑾嚙瞇pBest gBest
            for (int c1 = 0; c1 < guidePointArray.size(); c1++)
            {
                math_vector3d guidePoint = guidePointArray.get(c1);
                for (int c2 = 0; c2 < profilePointArray.size(); c2++)
                {
                    math_vector3d profilePoint = profilePointArray.get(c2);
                    distance = Math.abs(guidePoint.Distance(profilePoint));

                    if (distance < profilePFitness[c2])
                    {
                        profilePFitness[c2] = distance;
                        profilePBestU[c2] = profilePCurrentU[c2];
                    }
                    if (distance < profileGlobalDis)
                    {
                        profileGlobalDis = distance;
                        profileGBestU = profilePCurrentU[c2];
                    }

                    //
                    if (distance < guidePFitness[c1])
                    {
                        guidePFitness[c1] = distance;
                        guidePBestU[c1] = guidePCurrentU[c1];
                    }
                    if (distance < guideGlobalDis)
                    {
                        guideGlobalDis = distance;
                        guideGBestU = guidePCurrentU[c1];
                    }

                    if (distance < smallestDis) //Leon
                    {
                        if ((smallestDis - distance) < 0.001)
                        {
                            isNext = false;
                        }
                        smallestDis = distance;
                    }
                }
            }

            // 嚙瞎嚙緩嚙磊嚙瑾嚙瞇嚙誹群
            guidePointArray.clear();
            profilePointArray.clear();
            for (int c1 = 0; c1 < number; c1++)
            {
                // 嚙磊嚙瑾嚙瞇guide嚙緣嚙論參潘蕭
                guideV[c1] = w * guideV[c1] + const1
                        * ((float) rand1.nextInt(1000) / (float) randRange)
                        * (guidePBestU[c1] - guidePCurrentU[c1]) / deltaT + const2
                        * ((float) rand2.nextInt(1000) / (float) randRange)
                        * (guideGBestU - guidePCurrentU[c1]) / deltaT;
                // 嚙磊嚙瑾嚙瞇profile嚙緣嚙論參潘蕭
                profileV[c1] = w * profileV[c1] + const1
                        * ((float) rand1.nextInt(1000) / (float) randRange)
                        * (profilePBestU[c1] - profilePCurrentU[c1]) / deltaT
                        + const2
                        * ((float) rand2.nextInt(1000) / (float) randRange)
                        * (profileGBestU - profilePCurrentU[c1]) / deltaT;

                if (guideV[c1] > vupper)
                {
                    guideV[c1] = vupper;
                }
                else if (guideV[c1] < vlower)
                {
                    guideV[c1] = vlower;
                }

                if (profileV[c1] > vupper)
                {
                    profileV[c1] = vupper;
                }
                else if (profileV[c1] < vlower)
                {
                    profileV[c1] = vlower;
                }

                // 嚙磊嚙瑾嚙瞇guide嚙瞑profile
                guidePCurrentU[c1] = guidePCurrentU[c1] + guideV[c1] * deltaT;
                profilePCurrentU[c1] = profilePCurrentU[c1] + profileV[c1]
                        * deltaT;

                if (guidePCurrentU[c1] > 1)
                {
                    guidePCurrentU[c1] = 1;
                }
                else if (guidePCurrentU[c1] < 0)
                {
                    guidePCurrentU[c1] = 0;
                }
                if (profilePCurrentU[c1] > 1)
                {
                    profilePCurrentU[c1] = 1;
                }
                else if (profilePCurrentU[c1] < 0)
                {
                    profilePCurrentU[c1] = 0;
                }

                math_vector3d guidePoint = new math_vector3d();
                math_vector3d profilePoint = new math_vector3d();
                guide.Coordinate(guidePCurrentU[c1], guidePoint);
                profile.Coordinate(profilePCurrentU[c1], profilePoint);

                guidePointArray.add(guidePoint);
                profilePointArray.add(profilePoint);
            }
            counter++;
        }

        math_vector3d finalPoint = new math_vector3d();
        guide.Coordinate(guideGBestU, finalPoint);
        return finalPoint;
    }

//    public static void main(String[] args)
//    {
//        step_curve_array profileArray = ReadProfiles();
//        step_curve_array guideArray = ReadGuides();
//
//        prim_curve_network_test_1 network = new prim_curve_network_test_1(false, profileArray, guideArray);
//        step_b_spline_surface bsp = network.GetSumSurface();
//        bsp.WriteToSurfaceModel("D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSurface0003.stp");
//    }

    //20110923 yin ---------------------------
    public void SetFinalVector2()
    {

        ArrayList<Double> knotArrayOfGuide = new ArrayList<Double>();
        ArrayList<Double> knotArrayOfProfile = new ArrayList<Double>();
        double_array knotVector = new double_array();
        double_array insertArrayOfU = new double_array();
        double_array insertArrayOfV = new double_array();
        double_array removeArrayOfU;
        double_array removeArrayOfV;

        //for guide
        int Uorder = this.guideSurface.Uorder();
        for (int i = Uorder; i < this.guideSurface.UknotCount() - Uorder; i++)
        {
            double s;
            s = this.guideSurface.GetUknotArray().Get(i);
            knotArrayOfGuide.add(s);
            insertArrayOfU.Add(s);
        }

        //for profile
        int Vorder = this.profileSurface.Vorder();
        for (int i = Vorder; i < this.profileSurface.VknotCount() - Vorder; i++)
        {
            double s;
            s = this.profileSurface.GetVknotArray().Get(i);
            knotArrayOfProfile.add(s);
            insertArrayOfV.Add(s);
        }


        //change guide  v knot
        int removeVOrder = this.guideSurface.Vorder();
        removeArrayOfV = new double_array();
        for (int i = removeVOrder; i < this.guideSurface.VknotCount() - removeVOrder; i++)
        {
            double s;
            s = this.guideSurface.GetVknotArray().Get(i);
            removeArrayOfV.Add(s);
        }

        //first add
        this.guideSurface.InsertAndRemoveKnots(null, insertArrayOfV.ToDouble(), null, null);
        //second remove
        this.guideSurface.InsertAndRemoveKnots(null, null, null, removeArrayOfV.ToDouble());
        //change guide  v knot

        //change profile  u knot
        int removeUOrder = this.profileSurface.Uorder();
        removeArrayOfU = new double_array();
        for (int i = removeUOrder; i < this.profileSurface.UknotCount() - removeUOrder; i++)
        {
            double s;
            s = this.profileSurface.GetUknotArray().Get(i);
            removeArrayOfU.Add(s);
        }

        //first add
        this.profileSurface.InsertAndRemoveKnots(insertArrayOfU.ToDouble(), null, null, null);
        //second remove
        this.profileSurface.InsertAndRemoveKnots(null, null, removeArrayOfU.ToDouble(), null);
        //change guide  u knot

        //change tensorProductSurface  u  v knot
        removeArrayOfU = new double_array();
        removeArrayOfV = new double_array();

        int orderOfTU = this.tensorProductSurface.Uorder();
        for (int i = orderOfTU; i < this.tensorProductSurface.UknotCount() - orderOfTU; i++)
        {
            double s;
            s = this.tensorProductSurface.GetUknotArray().Get(i);
            removeArrayOfU.Add(s);
        }

        int orderOfTV = this.tensorProductSurface.Vorder();
        for (int i = orderOfTV; i < this.tensorProductSurface.VknotCount() - orderOfTV; i++)
        {
            double s;
            s = this.tensorProductSurface.GetVknotArray().Get(i);
            removeArrayOfV.Add(s);
        }

        //first add
        this.tensorProductSurface.InsertAndRemoveKnots(insertArrayOfU.ToDouble(), null, null, null);
        //second remove
        this.tensorProductSurface.InsertAndRemoveKnots(null, null, removeArrayOfU.ToDouble(), null);

        //first add
        this.tensorProductSurface.InsertAndRemoveKnots(null, insertArrayOfV.ToDouble(), null, null);
        //second remove
        this.tensorProductSurface.InsertAndRemoveKnots(null, null, null, removeArrayOfV.ToDouble());
        //change tensorProductSurface  u  v knot


    }
    //20110923 yin ---------------------------

    private void UpdateSumSurface()
    {
        math_upoint upoint;
        math_vector3d coord;
        step_b_spline_curve bsp;
        int num;
        math_vector3d old;
        bsp = (step_b_spline_curve) this.guideCurveArray.Get(0);
        upoint = this.sumSurface.GetControlPoint().Get(0);
        int count = upoint.GetPoint().Count();
        for (int i = 0; i < count; ++i)
        {
            coord = bsp.GetControlPoint(i).GetCoord();
            old = upoint.GetPoint(i).GetCoord();
            upoint.GetPoint(i).SetCoord(coord);
        }

        num = this.guideCurveArray.Count();
        bsp = (step_b_spline_curve) this.guideCurveArray.Get(num - 1);
        num = this.sumSurface.GetControlPoint().Count();
        upoint = this.sumSurface.GetControlPoint().Get(num - 1);
        count = upoint.GetPoint().Count();
        for (int i = 0; i < count; ++i)
        {
            coord = bsp.GetControlPoint(i).GetCoord();
            old = upoint.GetPoint(i).GetCoord();
            upoint.GetPoint(i).SetCoord(coord);
        }

        bsp = (step_b_spline_curve) this.profileCurveArray.Get(0);
        count = bsp.Ncontrol();
        for (int i = 0; i < count; ++i)
        {
            upoint = this.sumSurface.GetControlPoint().Get(i);

            coord = bsp.GetControlPoint(i).GetCoord();
            old = upoint.GetPoint(0).GetCoord();
            upoint.GetPoint(0).SetCoord(coord);
        }

        num = this.profileCurveArray.Count();
        bsp = (step_b_spline_curve) this.profileCurveArray.Get(num - 1);
        count = bsp.Ncontrol();
        for (int i = 0; i < count; ++i)
        {
            upoint = this.sumSurface.GetControlPoint().Get(i);

            coord = bsp.GetControlPoint(i).GetCoord();
            num = upoint.GetPoint().Count();
            old = upoint.GetPoint(num - 1).GetCoord();
            upoint.GetPoint(num - 1).SetCoord(coord);
        }
    }  /* end of procedure UpdateSumSurface */

}
