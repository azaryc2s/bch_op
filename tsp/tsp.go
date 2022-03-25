package tsp

import (
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"git.solver4all.com/azaryc2s/op"
	"log"
	"math"
	"os"
)

var (
	gurobiEnv *gurobi.Env
	subtours  [][]int32
	varCount  int
)

/* Define structure to pass data to the callback function */

type SubData struct {
	N int32
}

/* Given an integer-feasible solution 'sol', find the smallest sub-tour.  Result is returned in 'tour', and length is returned in 'tourlenP'. */

func findsubtour(edges [][]int) (result []int32) {
	n := int32(len(edges))
	seen := make([]bool, n)
	tour := make([]int32, n)

	start := int32(0)
	bestlen := n + 1
	bestind := int32(-1)
	i := int32(0)
	node := int32(0)
	for start < n {
		for node = 0; node < n; node++ {
			if !seen[node] {
				break
			}
		}
		if node == n {
			break
		}
		for leng := int32(0); leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if edges[node][i] == 1 && !seen[i] {
					node = i
					break
				}
			}
			if i == n {
				leng++
				if leng < bestlen {
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

/* Subtour elimination callback.  Whenever a feasible solution is found, find the shortest subtour and then add the subtour elimination constraint if that tour doesn't visit every node. */

func subtourelimTSP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	n := usrdata.(SubData).N

	if where == gurobi.CB_MIPSOL {
		sol, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, varCount)
		if err != nil {
			log.Println(err)
		}
		solA := extractEdgeMatrix(sol, int(n))
		tour := findsubtour(solA)
		if int32(len(tour)) < n {
			subtours = append(subtours, tour)
			var (
				ind []int32
				val []float64
			)

			/* Add a subtour elimination constraint */
			for i := 0; i < len(tour); i++ {
				for j := i + 1; j < len(tour); j++ {
					ind = append(ind, int32(op.GetEdgeIndex(int(tour[i]), int(tour[j]), int(n), 0)))
				}
			}
			for i := 0; i < len(ind); i++ {
				val = append(val, 1.0)
			}

			err = gurobi.CbLazy(cbdata, len(ind), ind, val, gurobi.LESS_EQUAL, float64(len(tour)-1))
			if err != nil {
				log.Println(err)
			}
		}
	}

	return 0
}

func extractEdgeMatrix(solA []float64, N int) [][]int {
	yMat := make([][]int, N)
	for i := 0; i < N; i++ {
		yMat[i] = make([]int, N)
	}
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			if solA[op.GetEdgeIndex(i, j, N, 0)] > 0.5 {
				yMat[i][j] = 1
				yMat[j][i] = 1
			}
		}
	}
	return yMat
}

/* Euclidean distance between points 'i' and 'j'. */

func distance(x, y []float64, i, j int) float64 {
	dx := x[i] - y[i]
	dy := x[j] - y[j]

	return math.Sqrt(dx*dx + dy*dy)
}

/*func getEdgeIndex(i, j, N int) int {
	if j < i {
		i, j = j, i
	}
	count := 0
	for k := 0; k < i; k++ {
		count += N - 1 - k
	}
	count += j - i - 1
	return count
}*/

func SolveTSP(d [][]int) ([]int32, int, [][]int32) {
	/* Reset variables and create the gurobi environment */
	var err error
	subtours = make([][]int32, 0)
	gurobiEnv, err = gurobi.LoadEnv("tsp_gurobi.log")
	varCount = 0
	N := len(d)
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}
	defer gurobiEnv.Free()

	gurobiEnv.SetIntParam("LogToConsole", int32(0))
	defer gurobiEnv.SetIntParam("LogToConsole", int32(1))

	/* Create an empty model */

	model, err := gurobiEnv.NewModel("tsp", 0, nil, nil, nil, nil, nil)
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}
	defer model.Free()

	/* Add variables X_ij - one for every pair of nodes where j > i weighted by the distance in the obj function*/
	{
		for i := 0; i < N; i++ {
			for j := i + 1; j < N; j++ {
				name := fmt.Sprintf("Y_%d_%d", i, j)
				err = model.AddVar(nil, nil, float64(d[i][j]), 0.0, 1.0, gurobi.BINARY, name)
				if err != nil {
					log.Println(err)
					return nil, -1, nil
				}
				varCount++
			}
		}
	}

	/* Restrict the variables so that the sum of edges for each node is always = 2*/
	{
		for i := 0; i < N; i++ {
			var (
				ind []int32
				val []float64
			)
			for j := i + 1; j < N; j++ {
				ind = append(ind, int32(op.GetEdgeIndex(i, j, N, 0)))
				val = append(val, 1.0)
			}
			for j := 0; j < i; j++ {
				ind = append(ind, int32(op.GetEdgeIndex(j, i, N, 0)))
				val = append(val, 1.0)
			}
			err = model.AddConstr(ind, val, gurobi.EQUAL, 2.0, fmt.Sprintf("node_2_%d", i))
			if err != nil {
				log.Printf("Error adding node_2_%d\n", i)
				log.Printf("At %s: %s\n", os.Args[1], err.Error())
				return nil, -1, nil
			}
		}
	}

	/* Set callback function */

	err = model.SetCallbackFuncGo(subtourelimTSP, SubData{N: int32(N)})
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}

	/* Must set LazyConstraints parameter when using lazy constraints */

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}

	/* Optimize model */

	err = model.Optimize()
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}

	/* Extract solution */
	solcount, err := model.GetIntAttr(gurobi.INT_ATTR_SOLCOUNT)
	if err != nil {
		log.Println(err)
		return nil, -1, nil
	}
	if solcount > 0 {
		sol, err := model.GetDblAttrArray(gurobi.DBL_ATTR_X, 0, int32(varCount))
		if err != nil {
			log.Println(err)
			return nil, -1, nil
		}
		solA := extractEdgeMatrix(sol, N)
		tour := findsubtour(solA)
		length := 0
		for i := 0; i < len(tour); i++ {
			j := (i + 1) % len(tour)
			length += d[int(tour[i])][int(tour[j])]
		}
		return tour, length, subtours
	}
	return nil, -1, nil
}
