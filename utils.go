package op

import (
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"math"
	"regexp"
)



func GetEdgeIndex(i, j, N, start int) int {
	if j < i {
		i, j = j, i
	}
	count := start
	for k := 0; k < i; k++ {
		count += N - 1 - k
	}
	count += j - i - 1
	return count
}

func CalcEdgeDist(coordinates [][]float64, distType string) [][]int {
	n := len(coordinates)
	result := make([][]int, n)
	for node := 0; node < n; node++ {
		result[node] = make([]int, n)
		for node2 := 0; node2 < node; node2++ {
			xDist := coordinates[node][0] - coordinates[node2][0]
			yDist := coordinates[node][1] - coordinates[node2][1]
			var distance int
			if distType == "EUC_2D" {
				distance = int(math.Sqrt(math.Pow(xDist, 2)+math.Pow(yDist, 2)) + 0.5)
			} else if distType == "CEIL_2D" {
				distance = int(math.Ceil(math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))))
			}
			result[node][node2] = distance
			result[node2][node] = distance
		}
	}
	return result
}

func GetSECs(subtours [][]int32, N int, start int) (secInd [][]int32, secVal [][]float64, op int8, rhs []float64) {
	for _, stour := range subtours {
		var (
			ind []int32
			val []float64
		)

		/* Add a subtour elimination constraint */
		for i := 0; i < len(stour); i++ {
			for j := i + 1; j < len(stour); j++ {
				if stour[j] > stour[i] {
					ind = append(ind, int32(GetEdgeIndex(int(stour[i]), int(stour[j]), N, start)))
				} else {
					ind = append(ind, int32(GetEdgeIndex(int(stour[j]), int(stour[i]), N, start)))
				}

			}
		}
		for i := 0; i < len(ind); i++ {
			val = append(val, 1.0)
		}

		secInd = append(secInd, ind)
		secVal = append(secVal, val)
		rhs = append(rhs, float64(len(stour)-1))
	}
	return secInd, secVal, gurobi.LESS_EQUAL, rhs
}


func Print2DArray(a [][]int) {
	for _, x := range a {
		for _, y := range x {
			fmt.Printf("%d,", y)
		}
		fmt.Println("")
	}
}

func SanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s", json)
	var numbers = regexp.MustCompile(`\s*([0-9]+),\s+([0-9]+)(,)?`)
	var brackets = regexp.MustCompile(`\[(([0-9]+,)+[0-9]+)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$2$3")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$3$4")
	}
	return res
}
