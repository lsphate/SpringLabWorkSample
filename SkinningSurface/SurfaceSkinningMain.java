package SkinningSurface;

import SpringSolid.Part42.step_b_spline_curve;
import SpringSolid.Part42.step_b_spline_surface;
import SpringSolid.Part42.step_composite_curve;
import SpringSolid.Part42.step_curve;
import SpringSolid.Part42.step_geometric_set;
import SpringSolid.Part42.step_read;
import SpringSolid.Part42Array.step_curve_array;

public class SurfaceSkinningMain implements SpringSolid.Part42.Define
{

    public static void main(String[] args)
    {
        String sectionPath = "D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSection01.stp";
        String spinePath = "D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSpine01.stp";
        String guidePath = "";
        String surfaceOutPath = "D:\\Graduate\\Computational Geometry\\TermProjectTest\\20120115\\SkinTestSurface01.stp";


        step_curve_array sectionArray = ReadSections(sectionPath);
        step_curve_array spineArray = ReadSpine(spinePath);
        step_curve_array guideArray = ReadGuides(guidePath);
        step_b_spline_curve[] sectionCurve = new step_b_spline_curve[sectionArray.Count()];
        step_b_spline_curve[] spineCurve = new step_b_spline_curve[spineArray.Count()];

        for (int i = 0; i < sectionArray.Size(); i++)
        {
            sectionCurve[i] = (step_b_spline_curve) sectionArray.Get(i);
        }
        spineCurve[0] = (step_b_spline_curve) spineArray.Get(0);

        SurfaceSkinning skin = new SurfaceSkinning(sectionCurve, spineCurve[0]);
        step_b_spline_surface surface = skin.GetSkinningSurface();
        surface.WriteToSurfaceModel(surfaceOutPath);
    }

    public static step_curve_array ReadSections(String path)
    {
        step_read read1 = new step_read();
        read1.ReadData(path);
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

    public static step_curve_array ReadSpine(String path)
    {
        step_read read1 = new step_read();
        read1.ReadData(path);
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

    public static step_curve_array ReadGuides(String path)
    {
        step_read read1 = new step_read();
        read1.ReadData(path);
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
}
