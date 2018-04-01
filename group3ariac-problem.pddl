(define (problem ariac_competition_problem)
  (:domain ariac_competition_domain)
  ;;objects
  (:objects
    gear_part_0 gear_part_1 gear_part_2 gear_part_3 
    piston_rod_part_0 piston_rod_part_1 piston_rod_part_2 - parttype
    ur10 - robot
    agvtray - agvtray
    bin1 bin2 - bintray
    order - order
    )

  ;;initial state
  (:init
    (=(No-of-parts-in-order order) 4)
    (orderlist order gear_part_0)
    (orderlist order gear_part_1) 
    (orderlist order piston_rod_part_0)
    (orderlist order piston_rod_part_1)        
    (parttypeOnbintray gear_part_0 bin1)
    (parttypeOnbintray gear_part_1 bin1)
    (parttypeOnbintray gear_part_2 bin1)
    (parttypeOnbintray gear_part_3 bin1)        
    (parttypeOnbintray piston_rod_part_0 bin2)
    (parttypeOnbintray piston_rod_part_1 bin2)
    (parttypeOnbintray piston_rod_part_2 bin2)    
    (handempty ur10)
    )

  (:goal (and
      (= (No-of-parts-in-order order) 0)
      )
    )

  )
