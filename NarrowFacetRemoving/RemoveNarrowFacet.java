//lsphate 2011/09/22 --------------- Remove Narrow Facet
    public void RemoveNarrowFacet()
    {
        //set all facetID to vertex
        for (int i = 0; i < facetArray.size(); i++)
        {
            mesh_facet facet = facetArray.get(i);

            for (int j = 0; j < 3; j++)
            {
                vertexArray.get(facet.GetVertexIDArray()[j]).SetFacetHaveVertex(facet.GetID());
            }
        }
//Step 1. Get all narrow facet.
        ArrayList<Integer> narrowFacet = new ArrayList<Integer>();//matrix fo narrow facet ID.
        narrowFacet = this.FindNarrowFacet();//find all narrow facet.
//Step 2. Find facet which is going to merge with target narrow facet and merge them.
        int targetMergeFacetID;//ID of facet which target narrow facet is going to merge in.
        ArrayList<mesh_facet> adjacentFacetOfNarrowFacet = new ArrayList<mesh_facet>();
        int countBorder = 0;
        ArrayList<mesh_facet> countBorderWithoutBorderAdjFacet = new ArrayList<mesh_facet>();
        ArrayList<mesh_facet> countBorderSingle = new ArrayList<mesh_facet>();
        ArrayList<Integer> countInner = new ArrayList<Integer>();
        for (int i = 0; i < narrowFacet.size(); i++)
        {
            adjacentFacetOfNarrowFacet = this.facetArray.get(narrowFacet.get(i)).FindAdjacentFacet(facetArray);
            if (adjacentFacetOfNarrowFacet.size() == 3)//target narrow facet is NOT at border.
            {
                countInner.add(narrowFacet.get(i));
            }
            else if (adjacentFacetOfNarrowFacet.size() == 2)//target border narrow facet has 2 adjacent facets.
            {
                targetMergeFacetID = this.GetBorderAdjacentFacet(narrowFacet.get(i),
                        this.facetArray.get(narrowFacet.get(i)).FindAdjacentFacet(facetArray));
                if (targetMergeFacetID != -1)//target merge facet is at border.
                {
                    this.MergeTwoFacetAtBorder(narrowFacet.get(i), targetMergeFacetID);
                    countBorder++;
                }
                else//target border narrow doesn't have adjacent facet which is at border.
                {
                    this.MergeThreeFacetAtBorder(narrowFacet.get(i));
                    countBorderWithoutBorderAdjFacet.add(this.facetArray.get(narrowFacet.get(i)));
                }
            }
            else//target border narrow facet has 1 adjacent facet.
            {
                countBorderSingle.add(this.facetArray.get(narrowFacet.get(i)));
                this.facetArray.get(narrowFacet.get(i)).SetDeleted();
            }

        }
        ArrayList<Integer> removableInnerFacet = this.CollectCanFitFacet(countInner);
        for (int i= 0; i < removableInnerFacet.size(); i++)
        {
            this.MeshSimplifyFacetInside(removableInnerFacet.get(i));
        }
//Step 3. Set deleted facet null and reset all facet data.
        for (int i = 0; i < facetArray.size(); i++)
        {
            if (this.facetArray.get(i).IsDeleted() == true)
            {
                this.facetArray.set(i, null);
            }
        }
        System.out.println(narrowFacet.size() + " narrow facets detected");
        System.out.println((countBorder + countBorderWithoutBorderAdjFacet.size() + countBorderSingle.size()) + " at border have been successfully removed.");
        //System.out.println(countBorderSingle.size() + " of them are single facet.");
        System.out.println(countInner.size() + " are inner narrow facets.");
        this.RebuildFacetArray();
        this.ResetAllData();
        //set vertex have facet null
        for (int i = 0; i < this.vertexArray.size(); i++)
        {
            vertexArray.get(i).SetFacetHaveVertex(-999);
        }
    }

//Find narrow facet from all facet.
    public ArrayList<Integer> FindNarrowFacet()
    {
        ArrayList<Integer> suspectNarrowFacet = new ArrayList<Integer>();//matrix for narrow facet suspects.
        ArrayList<Integer> narrowFacet = new ArrayList<Integer>();//matrix for narrow facet.
        int[] facetEdgeID = new int[3];
        float edgeLengthTotal = 0;
        double[] facetLength = new double[3];
        double lengthThreshold;
        double[] facetAngle = new double[3];
        double angleThreshold = 5;
        for (int i = 0; i < this.edgeArray.size(); i++)
        {
            edgeLengthTotal += this.edgeArray.get(i).GetLength();
        }
        lengthThreshold = (double) edgeLengthTotal / (this.edgeArray.size());//get length threshold value.
        for (int i = 0; i < facetArray.size(); i++)
        {
            facetEdgeID = this.facetArray.get(i).GetMeshID();
            facetAngle[0] = this.FacetEdgeAngle(facetEdgeID[0], facetEdgeID[1]);
            facetAngle[1] = this.FacetEdgeAngle(facetEdgeID[1], facetEdgeID[2]);
            facetAngle[2] = this.FacetEdgeAngle(facetEdgeID[2], facetEdgeID[0]);

            if (facetAngle[0] < angleThreshold || facetAngle[1] < angleThreshold || facetAngle[2] < angleThreshold)
            {
                suspectNarrowFacet.add(i);
            }
        }
        //method 1:check each mesh, fliter one whose angles are small than threshold, then save their ID in uspectF.
        for (int i = 0; i < suspectNarrowFacet.size(); i++)
        {
            this.edgeArray.get(suspectNarrowFacet.get(i)).SetVertex(this.vertexArray);//import vertex data into edgeArray method.
            facetEdgeID = this.facetArray.get(suspectNarrowFacet.get(i)).GetMeshID();
            for (int j = 0; j < 3; j++)
            {
                facetLength[j] = this.edgeArray.get(facetEdgeID[j]).GetLength();
            }
            if (facetLength[0] < lengthThreshold || facetLength[1] < lengthThreshold || facetLength[2] < lengthThreshold)
            {
                narrowFacet.add(suspectNarrowFacet.get(i));
            }
        }
        //method 2: check the length of the edges, fliter the short ones and save their ID in suspectF.
        math_vector3d_array findNarrowFacetTest = new math_vector3d_array();
        for (int i = 0; i < narrowFacet.size(); i++)
        {
            int narrowFacetVertexID[] = new int[3];
            narrowFacetVertexID = this.facetArray.get(narrowFacet.get(i)).GetVertexIDArray();
            for (int j = 0; j < 3; j++)
            {
                findNarrowFacetTest.Add(this.vertexArray.get(narrowFacetVertexID[j]).TransferMesh_VertexToMath_Vector3d());
            }
        }
        findNarrowFacetTest.WriteToSurfaceModel("D:\\Graduate\\Project\\narrow_facet.stp");//generate test stp file.

        System.out.println("Start-----------------------");
        System.out.println(narrowFacet);
        System.out.println("Total amount of narrow facets: " + narrowFacet.size());
        System.out.println("End-------------------------");//generate test text.
        return narrowFacet;
    }

//Obtain the biggest Adjacent facet ID.
    public ArrayList<Integer> GetTheBiggestAdjacentFacet(ArrayList<mesh_facet> adjacentFacet)
    {
        ArrayList<Integer> adjacentBigFacetID = new ArrayList<Integer>();
        ArrayList<Double> adjacentFacentArea = new ArrayList<Double>();
        for (int i = 0; i < adjacentFacet.size(); i++)
        {
            adjacentBigFacetID.add(adjacentFacet.get(i).GetID());
            adjacentFacentArea.add((double) adjacentFacet.get(i).GetArea());
        }//obtain adjacent facet area.
        double bubbleAreaTemp;
        int bubbleIDTemp;
        for (int i = 0; i < adjacentFacet.size(); i++)
        {
            for (int j = i; j < adjacentFacet.size(); j++)
            {
                if (adjacentFacentArea.get(i) < adjacentFacentArea.get(j))
                {
                    bubbleAreaTemp = adjacentFacentArea.get(i);
                    bubbleIDTemp = adjacentBigFacetID.get(i);
                    adjacentFacentArea.set(i, adjacentFacentArea.get(j));
                    adjacentBigFacetID.set(i, adjacentBigFacetID.get(j));
                    adjacentFacentArea.set(j, bubbleAreaTemp);
                    adjacentBigFacetID.set(j, bubbleIDTemp);
                }
            }
        }//bubble sort to find the biggest facet.
        return adjacentBigFacetID;
    }

//Obtain the adjacent facet which is at border.
    public int GetBorderAdjacentFacet(int removeFacetID, ArrayList<mesh_facet> adjacentFacets)
    {
        ArrayList<Integer> adjacentFacetsID = new ArrayList<Integer>();
        ArrayList<Integer> numOfAdjacentFacet = new ArrayList<Integer>();
        int borderFacet = -1;
        ArrayList<mesh_facet> adjFacetOfAdjFacets = new ArrayList<mesh_facet>();
        for (int i = 0; i < adjacentFacets.size(); i++)
        {
            adjacentFacetsID.add(adjacentFacets.get(i).GetID());
        }
        for (int i = 0; i < adjacentFacetsID.size(); i++)
        {
            adjFacetOfAdjFacets = this.facetArray.get(adjacentFacetsID.get(i)).FindAdjacentFacet(facetArray);
            numOfAdjacentFacet.add(adjFacetOfAdjFacets.size());
        }
        for (int i = 0; i < numOfAdjacentFacet.size(); i++)
        {
            if (numOfAdjacentFacet.get(i) == 1 && adjacentFacetsID.get(i) != removeFacetID)
            {
                borderFacet = adjacentFacetsID.get(i);
                break;
            }
            else if (numOfAdjacentFacet.get(i) == 2 && adjacentFacetsID.get(i) != removeFacetID)
            {
                borderFacet = adjacentFacetsID.get(i);
            }
        }
        return borderFacet;
    }

//Merge two facet both are at border.
    public void MergeTwoFacetAtBorder(int removeFacetID, int keepFacetID)
    {
        mesh_vertex deleteVertex = this.vertexArray.get(0);
        int[] finalFacetVertexID = new int[3];
        deleteVertex = this.GetDeleteVertexAtBorder(removeFacetID, keepFacetID);
        finalFacetVertexID = this.GetFinalFacetVertexAtBorder(removeFacetID, keepFacetID, deleteVertex);
        this.CreatFacetByThreeVertex(finalFacetVertexID[0], finalFacetVertexID[1], finalFacetVertexID[2]);
        this.facetArray.get(removeFacetID).SetDeleted();
        this.facetArray.get(keepFacetID).SetDeleted();
        //build data of fianl facet.
    }

    public void CreatFacetByThreeVertex(int vertexID1, int vertexID2, int vertexID3)
    {
        mesh_facet finalFacet = new mesh_facet(this);
        finalFacet.SetFacetNormal(this.ThreePointsFindNormal(
                this.vertexArray.get(vertexID1).TransferMesh_VertexToMath_Vector3d(),
                this.vertexArray.get(vertexID2).TransferMesh_VertexToMath_Vector3d(),
                this.vertexArray.get(vertexID3).TransferMesh_VertexToMath_Vector3d()));
        finalFacet.SetVertexID(0, vertexID1);
        finalFacet.SetVertexID(1, vertexID2);
        finalFacet.SetVertexID(2, vertexID3);
        finalFacet.SetID(this.facetArray.size());
        this.facetArray.add(finalFacet);
    }

    public mesh_vertex GetDeleteVertexAtBorder(int removeFacetID, int keepFacetID)
    {
        int[] removeFacetVertexID = this.facetArray.get(removeFacetID).GetVertexIDArray();
        int[] keepFacetVertexID = this.facetArray.get(keepFacetID).GetVertexIDArray();
        ArrayList<mesh_vertex> removeFacetVertex = new ArrayList<mesh_vertex>();
        ArrayList<mesh_vertex> keepFacetVertex = new ArrayList<mesh_vertex>();
        for (int i = 0; i < 3; i++)
        {
            removeFacetVertex.add(this.vertexArray.get(removeFacetVertexID[i]));
            keepFacetVertex.add(this.vertexArray.get(keepFacetVertexID[i]));
        }
        //obtain vertex data of remove facet & merge facet.

        int[] removeFacetEdgeID = this.facetArray.get(removeFacetID).GetMeshID();
        int[] keepFacetEdgeID = this.facetArray.get(keepFacetID).GetMeshID();
        ArrayList<mesh_edge> removeFacetEdge = new ArrayList<mesh_edge>();
        ArrayList<mesh_edge> keepFacetEdge = new ArrayList<mesh_edge>();
        for (int i = 0; i < 3; i++)
        {
            removeFacetEdge.add(this.edgeArray.get(removeFacetEdgeID[i]));
            keepFacetEdge.add(this.edgeArray.get(keepFacetEdgeID[i]));
        }
        //obtain edge data of remove facet & merge facet.

        mesh_vertex tempVertex = this.vertexArray.get(0);
        mesh_vertex deleteVertex = this.vertexArray.get(0);
        for (int i = 0; i < 3; i++)
        {
            if (keepFacetVertexID[i] != removeFacetVertexID[0]
                    && keepFacetVertexID[i] != removeFacetVertexID[1]
                    && keepFacetVertexID[i] != removeFacetVertexID[2])
            {
                tempVertex = this.vertexArray.get(keepFacetVertexID[i]);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            if (keepFacetEdge.get(i).GetAdjEdgeID() < 0.0)
            {
                if (keepFacetEdge.get(i).GetStartVertexID() == tempVertex.GetID())
                {
                    deleteVertex = this.vertexArray.get(keepFacetEdge.get(i).GetEndVertexID());
                }
                else
                {
                    deleteVertex = this.vertexArray.get(keepFacetEdge.get(i).GetStartVertexID());
                }
            }
        }
        return deleteVertex;
    }

    public int[] GetFinalFacetVertexAtBorder(int removeFacetID, int keepFacetID, mesh_vertex deleteVertex)
    {
        ArrayList<mesh_edge> finalFacetEdge = new ArrayList<mesh_edge>();
        int[] finalFacetVertexID = new int[3];

        int[] removeFacetEdgeID = this.facetArray.get(removeFacetID).GetMeshID();
        int[] keepFacetEdgeID = this.facetArray.get(keepFacetID).GetMeshID();
        ArrayList<mesh_edge> removeFacetEdge = new ArrayList<mesh_edge>();
        ArrayList<mesh_edge> keepFacetEdge = new ArrayList<mesh_edge>();
        for (int i = 0; i < 3; i++)
        {
            removeFacetEdge.add(this.edgeArray.get(removeFacetEdgeID[i]));
            keepFacetEdge.add(this.edgeArray.get(keepFacetEdgeID[i]));
        }
        //obtain edge data of remove facet & merge facet.       
        for (int i = 0; i < 3; i++)
        {
            if (removeFacetEdge.get(i).GetStartVertexID() != deleteVertex.GetID()
                    && removeFacetEdge.get(i).GetEndVertexID() != deleteVertex.GetID())
            {
                finalFacetEdge.add(removeFacetEdge.get(i));
            }
        }
        //obtain edge which going to be removed from remove facet.
        for (int i = 0; i < 3; i++)
        {
            if (keepFacetEdge.get(i).GetStartVertexID() != deleteVertex.GetID()
                    && keepFacetEdge.get(i).GetEndVertexID() != deleteVertex.GetID())
            {
                finalFacetEdge.add(keepFacetEdge.get(i));
            }
        }
        //obtain edge which going to be removed from keep facet.        
        finalFacetVertexID[0] = finalFacetEdge.get(0).GetStartVertexID();
        finalFacetVertexID[1] = finalFacetEdge.get(0).GetEndVertexID();
        if (finalFacetEdge.get(1).GetStartVertexID() == finalFacetVertexID[1])
        {
            finalFacetVertexID[2] = finalFacetEdge.get(1).GetEndVertexID();
        }
        else
        {
            finalFacetVertexID[2] = finalFacetEdge.get(1).GetStartVertexID();
        }
        //obtain three vertex ID of final facet.

        return finalFacetVertexID;
    }

//Merge narrow facet with other two facet.
    public void MergeThreeFacetAtBorder(int removeFacetID)
    {
        mesh_facet[] finalTwoFacet = this.GetFinalTwoMergeFacet(removeFacetID);
        mesh_vertex deleteVertex;
        int[] finalFourVertexID = new int[4];
        if (finalTwoFacet[0] != null)
        {
            deleteVertex = this.ThreeFacetFindIntersectionVertex(removeFacetID, finalTwoFacet[0], finalTwoFacet[1]);
            finalFourVertexID = this.GetFinalFourVertexForTwoFacet(removeFacetID, finalTwoFacet[0], finalTwoFacet[1], deleteVertex);
            if (this.CheckVertexOrder(finalTwoFacet[1], deleteVertex) == 1)//border's at below, remove facet is at the left.
            {
                this.CreatFacetByThreeVertex(finalFourVertexID[0], finalFourVertexID[1], finalFourVertexID[2]);
                this.CreatFacetByThreeVertex(finalFourVertexID[0], finalFourVertexID[3], finalFourVertexID[1]);
            }
            else if (this.CheckVertexOrder(finalTwoFacet[1], deleteVertex) == 2)//border's at below, remove facet is at the right.
            {
                this.CreatFacetByThreeVertex(finalFourVertexID[0], finalFourVertexID[1], finalFourVertexID[2]);
                this.CreatFacetByThreeVertex(finalFourVertexID[0], finalFourVertexID[2], finalFourVertexID[3]);
            }
            this.facetArray.get(removeFacetID).SetDeleted();
            if (finalTwoFacet[0].IsDeleted() == false)
            {
                this.facetArray.get(finalTwoFacet[0].GetID()).SetDeleted();
            }
            if (finalTwoFacet[1].IsDeleted() == false)
            {
                this.facetArray.get(finalTwoFacet[1].GetID()).SetDeleted();
            }
        }

    }
//Get final two facet.

    public mesh_facet[] GetFinalTwoMergeFacet(int removeFacetID)
    {
        mesh_facet[] finalTwoFacet = new mesh_facet[2];//final two facet to merge.
        mesh_facet removeFacet = this.facetArray.get(removeFacetID);//target narrow facet(facet A).
        ArrayList<mesh_facet> adjacentFacet = removeFacet.FindAdjacentFacet(facetArray);//adjacent facet of narrow facet.
        mesh_facet targetFacet = this.facetArray.get(this.GetTheBiggestAdjacentFacet(adjacentFacet).get(0));//two choices of facet B.

        if (targetFacet.IsDeleted() == true)
        //first choice is a delete facet.
        {
            targetFacet = this.facetArray.get(this.GetTheBiggestAdjacentFacet(adjacentFacet).get(1));//jump to second choice.
            if (targetFacet.IsDeleted() == true)
            //second choice is also a delete facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)) == -1)
            //second choice doesn't have a border adjacent facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray))).IsDeleted() == true)
            //second choice's border adjacent facet is deleted.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else
            //second choice is NOT a delete facet and has border facet is also NOT delete.
            {
                finalTwoFacet[0] = targetFacet;//target is second choice.
                finalTwoFacet[1] = this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)));
            }
        }
        else if (this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)) == -1)
        //first choice doesn't have a border adjacent facet.
        {
            targetFacet = this.facetArray.get(this.GetTheBiggestAdjacentFacet(adjacentFacet).get(1));//jump to second choice.
            if (targetFacet.IsDeleted() == true)
            //second choice is a delete facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)) == -1)
            //second choice doesn't have a border adjacent facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray))).IsDeleted() == true)
            //second choice's border adjacent facet is deleted.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else
            //second choice is NOT a delete facet and has border facet is also NOT delete.
            {
                finalTwoFacet[0] = targetFacet;//target is second choice.
                finalTwoFacet[1] = this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)));
            }
        }
        else if (this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray))).IsDeleted() == true)
        //first choice's border adjacent facet is deleted.
        {
            targetFacet = this.facetArray.get(this.GetTheBiggestAdjacentFacet(adjacentFacet).get(1));//jump to second choice.
            if (targetFacet.IsDeleted() == true)//second choice of B is also a delete facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)) == -1)
            //second choice of B doesn't have a border adjacent facet.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else if (this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray))).IsDeleted() == true)
            //second choice's border facet is deleted.
            {
                finalTwoFacet[0] = null;//both two facets are not feasible.
                finalTwoFacet[1] = null;
            }
            else
            //second choice is NOT a delete facet and has border facet is also NOT delete.
            {
                finalTwoFacet[0] = targetFacet;//target is second choice.
                finalTwoFacet[1] = this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)));
            }
        }
        else
        //first choice is NOT a delete facet and has a border facet is NOT deleted.
        {
            finalTwoFacet[0] = targetFacet;//target is first choice.
            finalTwoFacet[1] = this.facetArray.get((int) this.GetBorderAdjacentFacet(removeFacetID, targetFacet.FindAdjacentFacet(facetArray)));
        }
        return finalTwoFacet;
    }
//Get the intersection vertex for three facet.

    public mesh_vertex ThreeFacetFindIntersectionVertex(int removeFacetID, mesh_facet midFacet, mesh_facet borderFacet)
    {
        mesh_vertex deleteVertex = this.vertexArray.get(0);
        int[] count = new int[3];
        int[] removeFacetVertexID = this.facetArray.get(removeFacetID).GetVertexIDArray();
        int[] midFacetVertexId = midFacet.GetVertexIDArray();
        int[] borderFacetVertexID = borderFacet.GetVertexIDArray();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (midFacetVertexId[j] == removeFacetVertexID[i])
                {
                    count[i]++;
                }
                if (borderFacetVertexID[j] == removeFacetVertexID[i])
                {
                    count[i]++;
                }
            }
        }
        for (int i = 0; i < 3; i++)
        {
            if (count[i] == 2)
            {
                deleteVertex = this.vertexArray.get(removeFacetVertexID[i]);
            }
        }
        return deleteVertex;
    }
//Get the fianl four vertex ID for final two facet.

    public int[] GetFinalFourVertexForTwoFacet(int removeFacetID, mesh_facet midFacet, mesh_facet borderFacet, mesh_vertex deleteVertex)
    {
        int[] finalFourVertexID = new int[4];
        int[] removeFacetVertexID = this.facetArray.get(removeFacetID).GetVertexIDArray();
        int[] midFacetVertexID = midFacet.GetVertexIDArray();
        int[] midFacetEdgeID = midFacet.GetMeshID();

        int[] borderFacetVertexID = borderFacet.GetVertexIDArray();
        for (int i = 0; i < 3; i++)
        {
            if (removeFacetVertexID[i] != midFacetVertexID[0]
                    && removeFacetVertexID[i] != midFacetVertexID[1]
                    && removeFacetVertexID[i] != midFacetVertexID[2])
            {
                finalFourVertexID[0] = removeFacetVertexID[i];
            }
        }
        //get vertex 1.
        for (int i = 0; i < 3; i++)
        {
            if (deleteVertex.GetID() != this.edgeArray.get(midFacetEdgeID[i]).GetStartVertexID()
                    && deleteVertex.GetID() != this.edgeArray.get(midFacetEdgeID[i]).GetEndVertexID())
            {
                finalFourVertexID[1] = this.edgeArray.get(midFacetEdgeID[i]).GetStartVertexID();
                finalFourVertexID[2] = this.edgeArray.get(midFacetEdgeID[i]).GetEndVertexID();

            }
        }
        //get vertex 2 & 3.
        for (int i = 0; i < 3; i++)
        {
            if (borderFacetVertexID[i] != midFacetVertexID[0]
                    && borderFacetVertexID[i] != midFacetVertexID[1]
                    && borderFacetVertexID[i] != midFacetVertexID[2])
            {
                finalFourVertexID[3] = borderFacetVertexID[i];
            }
        }
        //get vertex 4.
        return finalFourVertexID;
    }
//Check vertex order of four final vertex

    public int CheckVertexOrder(mesh_facet borderFacet, mesh_vertex deleteVertex)
    {
        int[] borderFacetEdgeID = borderFacet.GetMeshID();
        mesh_edge borderFacetBorderEdge = this.edgeArray.get(0);
        int judge = 0;
        for (int i = 0; i < 3; i++)
        {
            if (this.edgeArray.get(borderFacetEdgeID[i]).GetAdjEdgeID() < 0)
            {
                borderFacetBorderEdge = this.edgeArray.get(borderFacetEdgeID[i]);
            }
        }
        if (deleteVertex.GetID() == borderFacetBorderEdge.GetStartVertexID())
        {
            judge = 1;
        }
        else if (deleteVertex.GetID() == borderFacetBorderEdge.GetEndVertexID())
        {
            judge = 2;
        }
        return judge;
    }

//Calculate angle of two edge in one facet
    public double FacetEdgeAngle(int edge1ID, int edge2ID)
    {
        math_vector3d edge1_vector = this.edgeArray.get(edge1ID).GetVector();
        math_vector3d edge2_vector = this.edgeArray.get(edge2ID).GetVector().Minus();
        double dotproduct = edge1_vector.DotProduct(edge2_vector);
        double edge1_length = edge1_vector.Length();
        double edge2_length = edge2_vector.Length();
        double theta = (Math.acos(dotproduct / (edge1_length * edge2_length)) / Math.PI) * 180.0;
        return theta;
    }

//Obtain Edge IDs for Facet.
    public int[] ThreeEdgeOfFacet(int facetID)
    {
        int[] facetEdgeID = new int[3];
        facetEdgeID[0] = this.facetArray.get(facetID).GetRootEdgeID();
        facetEdgeID[1] = this.edgeArray.get(facetEdgeID[0]).GetNextEdgeID();
        facetEdgeID[2] = this.edgeArray.get(facetEdgeID[1]).GetNextEdgeID();
        return facetEdgeID;
    }

//Rebuild facetArray.
    public void RebuildFacetArray()
    {
        ArrayList<mesh_facet> newFacetArray = new ArrayList<mesh_facet>();
        for (int i = 0; i < this.facetArray.size(); i++)
        {
            mesh_facet newFacet = this.facetArray.get(i);
            if (newFacet != null)
            {
                newFacet.SetID(newFacetArray.size());
                newFacetArray.add(newFacet);
            }
        }
        this.facetArray = newFacetArray;
    }

    public ArrayList<Integer> CollectCanFitFacet(ArrayList<Integer> facetID)
    {
        //eliminate the region has edgeline and the overlapping facet
        ArrayList<Integer> canFitFacet = new ArrayList<Integer>();
        for (int i = 0; i < facetID.size(); i++)
        {
            ArrayList<Integer> useFacet = this.CollectAllUsedFacet(facetID.get(i));
            int edgeline = this.JudgeEdgeLine(useFacet);

            if (this.CheckFacetOverlap(facetID.get(i)) == false && edgeline == 0)
            {
                canFitFacet.add(facetID.get(i));
            }
        }
        int min = canFitFacet.get(0);
        ArrayList<Integer> allusedFacet = new ArrayList<Integer>();
        allusedFacet = this.CollectAllUsedFacet(min);
        for (int i = 0; i < allusedFacet.size(); i++)
        {
            facetArray.get(allusedFacet.get(i)).SetDeleted();
        }
        ArrayList<Integer> candoFacet = new ArrayList<Integer>();//Distinguish the regional
        candoFacet.add(min);
        for (int i = 1; i < canFitFacet.size(); i++)
        {
            allusedFacet = this.CollectAllUsedFacet(canFitFacet.get(i));
            int count = 0;
            for (int j = 0; j < allusedFacet.size(); j++)
            {
                if (facetArray.get(allusedFacet.get(j)).IsDeleted() == true)
                {
                    break;
                }
                else
                {
                    count++;
                }
            }
            if (count == allusedFacet.size())
            {
                candoFacet.add(canFitFacet.get(i));
                for (int k = 0; k < allusedFacet.size(); k++)
                {
                    facetArray.get(allusedFacet.get(k)).SetDeleted();
                }
            }
        }
        return candoFacet;
    }
//lsphate 2011/09/22 --------------- Remove Narrow Facet























    //20110927 lsphate
    private JMenuItem jMenuItemRemoveNarrowFacet = new JMenuItem("Remove Narrow Facet");
    //20110927 lsphate


        else if (e.getSource() == jMenuItemRemoveNarrowFacet)
        {
            System.out.println("Remove narrow facet .... please wait");

            long time = -System.currentTimeMillis();
            time = -System.currentTimeMillis();

            this.model.RemoveNarrowFacet();
            jMenuItemNewMesh.setEnabled(true);

            System.out.println("Remove narrow facet time:" + (time + System.currentTimeMillis()) / 1000f + "¬í");
        }