(define (domain ariac_competition_domain)

    (:requirements :strips :typing :fluents)

    (:types
        robot
        bintray
        agvtray
        parttype
        order
    )

    (:functions
        (No-of-parts-on-agv ?agvtray - agvtray)
        (No-of-parts-in-order ?order - order)  
    )

    (:predicates
        (orderlist ?order - order ?parttype - parttype)
        (parttypeOnbintray ?parttype - parttype ?bintray - bintray  )
        (handempty ?robot - robot)
        (pickpart ?bintray - bintray ?parttype - parttype)
        (partpicked ?robot - robot ?parttype - parttype )
        (partdropped ?parttype - parttype ?agvtray - agvtray )
        (robotoverbin ?robot - robot ?bintray - bintray )
        (robotoveragv ?robot - robot ?agvtray - agvtray )

    )

    (:action move-over-bin
        :parameters(
            ?order - order
            ?parttype - parttype
            ?robot - robot
            ?bintray - bintray)
        :precondition(and
            (handempty ?robot)
            ( orderlist ?order ?parttype)
            (parttypeOnbintray ?parttype ?bintray))
        :effect(and
            (robotoverbin ?robot ?bintray)
            (pickpart ?bintray ?parttype))
    )

    (:action move-over-agv
        :parameters(
            ?robot - robot
            ?bintray - bintray
            ?parttype - parttype
            ?agvtray - agvtray)
        :precondition(and
            (partpicked ?robot ?parttype)
            (robotoverbin ?robot ?bintray))
        :effect(and
            (not(pickpart ?bintray ?parttype))
            (robotoveragv ?robot ?agvtray)
            (not(handempty ?robot)))
     )

    (:action pickup
        :parameters(
            ?robot - robot
            ?parttype - parttype
            ?bintray - bintray)
        :precondition(and
            (pickpart ?bintray ?parttype)
            (robotoverbin ?robot ?bintray))
        :effect(and
            (not(pickpart ?bintray ?parttype))
            (partpicked ?robot ?parttype  )
            (not(handempty ?robot)))
    )
    (:action putdown
        :parameters(
            ?order - order
            ?robot - robot
            ?agvtray - agvtray
            ?parttype - parttype)
        :precondition(and
            (partpicked ?robot ?parttype  )
            (robotoveragv ?robot ?agvtray))
        :effect(and
            (decrease (No-of-parts-in-order ?order) 1)
            (increase (No-of-parts-on-agv ?agvtray) 1)
            (not( orderlist ?order ?parttype))
            (not(partpicked ?robot ?parttype  ))
            (partdropped ?parttype ?agvtray)
            (not(robotoveragv ?robot ?agvtray))
            (handempty ?robot))
    )
)
