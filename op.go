package op

import (
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"log"
)

/* Define structure to pass data to the callback function */

type SubData struct {
	N        int
	varCount int
	startX   int
	startY   int
}

func SolveOP(d [][]int, p []int, tmax int) (tour []int, score int, length int, optimstatus int32, lb int, err error) {
	// Create environment
	env, err := gurobi.LoadEnv("op-lp-sym.log")
	if err != nil {
		log.Printf("Error: %s\n", err.Error())
		return nil, -1, -1, -1, -1, err
	}
	defer env.Free()
	env.SetIntParam("LogToConsole", int32(0))
	defer env.SetIntParam("LogToConsole", int32(1))

	N := len(d)

	/* Create an empty model */

	model, err := env.NewModel("op", 0, nil, nil, nil, nil, nil)
	if err != nil {
		log.Println(err)
		return nil, -1, -1, -1, -1, err
	}
	defer model.Free()

	/* Add variables X_i - one for every node*/
	//log.Println("Adding variables X_i...")
	startX := 0
	varCount := 0
	for i := 0; i < N; i++ {
		name := fmt.Sprintf("X_%d", i)
		err = model.AddVar(nil, nil, float64(p[i]), 0.0, 1.0, gurobi.BINARY, name)
		if err != nil {
			log.Println(err)
			return nil, -1, -1, -1, -1, err
		}
		varCount++
	}
	startY := varCount
	/* Add variables Y_ij - one for every pair of nodes where j > i*/
	//log.Println("Adding variables Y_i_j...")
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			name := fmt.Sprintf("Y_%d_%d", i, j)
			err = model.AddVar(nil, nil, 0.0, 0.0, 1.0, gurobi.BINARY, name)
			if err != nil {
				log.Println(err)
				return nil, -1, -1, -1, -1, err
			}
			varCount++
		}
	}

	// Change objective sense to maximization
	err = model.SetIntAttr(gurobi.INT_ATTR_MODELSENSE, gurobi.MAXIMIZE)
	if err != nil {
		log.Printf("Error: %s\n", err.Error())
		return nil, -1, -1, -1, -1, err
	}

	//log.Println("Creating and setting a constraint for the depot 0 to always be used")
	{
		ind := []int{startX}
		val := []float64{1.0}
		name := fmt.Sprintf("must_depot")
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, name)
		if err != nil {
			log.Println("Error adding must_depot")
			return nil, -1, -1, -1, -1, err
		}
	}

	//log.Println("Creating and setting constraints for nodes to always be connected to 2 active edges")
	{
		for i := 0; i < N; i++ {
			var (
				ind []int32
				val []float64
			)
			for j := i + 1; j < N; j++ {
				ind = append(ind, int32(GetEdgeIndex(i, j, N, startY)))
				val = append(val, 1.0)
			}
			for j := 0; j < i; j++ {
				ind = append(ind, int32(GetEdgeIndex(j, i, N, startY)))
				val = append(val, 1.0)
			}
			ind = append(ind, int32(startX+i)) //X_i
			val = append(val, -2.0)
			err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("node_2_%d", i))
			if err != nil {
				log.Printf("Error adding node_2_%d\n", i)
				return nil, -1, -1, -1, -1, err
			}
		}
	}

	//log.Println("Creating and setting constraint for Tmax")
	{
		var (
			ind []int32
			val []float64
		)
		for i := 0; i < N; i++ {
			for j := i + 1; j < N; j++ {
				ind = append(ind, int32(GetEdgeIndex(i, j, N, startY)))
				val = append(val, float64(d[i][j]))
			}
		}
		err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, float64(tmax), "travel_budget")
		if err != nil {
			log.Printf("Error adding constraint for travel budget: %s\n", err.Error())
			return nil, -1, -1, -1, -1, err
		}
	}

	/* Set callback function */
	cbData := SubData{N: N, varCount: varCount, startX: startX, startY: startY}
	err = model.SetCallbackFuncGo(subtourelimOP, &cbData)
	if err != nil {
		log.Println(err)
		return nil, -1, -1, -1, -1, err
	}

	/* Must set LazyConstraints parameter when using lazy constraints */

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return nil, -1, -1, -1, -1, err
	}

	// Optimize model
	err = model.Optimize()
	if err != nil {
		log.Printf("Error: %s\n", err.Error())
		return nil, -1, -1, -1, -1, err
	}
	log.Println("\n---OPTIMIZATION DONE---\n")

	// Capture solution information
	optimstatus, err = model.GetIntAttr(gurobi.INT_ATTR_STATUS)
	if err != nil {
		log.Printf("Error capturing solution: %s\n", err.Error())
		return nil, -1, -1, -1, -1, err
	}

	objval, err := model.GetDblAttr(gurobi.DBL_ATTR_OBJVAL)
	if err != nil {
		log.Printf("Couldn't retrieve the obj-value: %s.\n", err.Error())
		return nil, -1, -1, -1, -1, err
	}
	score = int(objval + 0.5)

	lbF := 0.0
	lbF, err = model.GetDblAttr(gurobi.DBL_ATTR_OBJBOUND)
	if err != nil {
		log.Printf("Couldn't retrieve the lower-bound-value: %s. ", err.Error())
		return nil, -1, -1, -1, -1, err
	}
	lb = int(lbF + 0.5)

	solM, err := model.GetDblAttrArray(gurobi.DBL_ATTR_X, 0, int32(varCount))
	if err != nil {
		log.Println(err)
	}
	yMat := extractIntYSolution(solM, N, startY)
	//fmt.Printf("Found Solution: %v \n", solM)
	solNodes := 0
	opLength := 0
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			if yMat[i][j] == 1 { //this count edges set to 1. Having n such edges, means we have n nodes to visit
				solNodes++
				opLength += d[i][j]
			}
		}
	}
	tour = findsubtour(yMat)
	length = opLength
	//fmt.Printf("Found a op-tour with %d nodes and length %d: %v \n", len(tour), opLength, tour)
	return tour, score, length, optimstatus, lb, nil
}

/* Given an integer-feasible solution 'sol', find the smallest
   sub-tour.  Result is returned in 'tour', and length is
   returned in 'tourlenP'. */

func findsubtour(sol [][]int) (result []int) {
	n := len(sol)
	seen := make([]bool, n)
	tour := make([]int, n)

	start := 0
	bestlen := n + 1
	bestind := -1
	i := 0
	node := 0
	for start < n {
		for node = 0; node < n; node++ {
			if !seen[node] {
				break
			}
		}
		if node == n {
			break
		}
		isConnected := false
		for leng := 0; leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if sol[node][i] == 1 && !seen[i] {
					node = i
					isConnected = true
					break
				}
			}
			if i == n {
				leng++
				if isConnected && leng < bestlen {
					bestlen = leng
					bestind = start
				}
				start += leng
				break
			}
		}
	}

	return tour[bestind : bestind+bestlen]

}

/* Subtour elimination callback.  Whenever a feasible solution is found,
   find the shortest subtour, and add a subtour elimination constraint
   if that tour doesn't visit every node. */

func subtourelimOP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	cbData := usrdata.(*SubData)
	varCount := cbData.varCount
	N := cbData.N
	startX := cbData.startX
	startY := cbData.startY
	if where == gurobi.CB_MIPSOL {
		subSol, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, varCount)
		if err != nil {
			log.Println(err)
		}
		yMat := extractIntYSolution(subSol, N, startY)
		//fmt.Printf("Found subsolution: %v \n", subSol)
		subSolNodes := 0
		for i := startX; i < startY; i++ {
			if subSol[i] > 0.5 { //Counting the nodes to be visited
				subSolNodes++
			}
		}
		//fmt.Printf("We should be looking for subtours with less than %d nodes\n", subSolNodes)
		tour := findsubtour(yMat)

		if len(tour) < subSolNodes {
			//fmt.Printf("Found a subtour with %d nodes\n", len(tour))
			//fmt.Printf("%v\n", tour)
			secInd, secVal, oper, rhs := GetSECs([][]int32{gurobi.Int32Slice(tour)}, N, startY)

			for i := 0; i < len(secInd); i++ {
				//log.Printf("Adding SEC: %v * %v <= %.2f\n", secVal[i], secInd[i], rhs[i])
				err = gurobi.CbLazy(cbdata, len(secInd[i]), secInd[i], secVal[i], oper, rhs[i])
				if err != nil {
					log.Println(err)
				}
			}

		} else {
			//fmt.Printf("Found a valid tour with %d nodes\n", len(tour))
			//fmt.Printf("%v\n", tour)
		}
	}
	return 0
}

func extractIntYSolution(solA []float64, N int, startY int) [][]int {
	yMat := make([][]int, N)
	for i := 0; i < N; i++ {
		yMat[i] = make([]int, N)
	}
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			if solA[GetEdgeIndex(i, j, N, startY)] > 0.95 {
				yMat[i][j] = 1
				yMat[j][i] = 1
			}
		}
	}
	return yMat
}

func GetTourLength(tour []int, d [][]int) int{
	length := 0
	for i := 0; i < len(tour); i++{
		j := (i + 1) % len(tour)
		u := tour[i]
		v := tour[j]
		length += d[u][v]
	}
	return length
}
