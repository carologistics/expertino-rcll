(define (problem blocksworld-problem)
    (:domain blocksworld)

    (:objects
        block1 block2 block3 block4 - block
        robot1 - robot
    )

    (:init
        (clear block1)
        (clear block2)
        (clear block3)
        (clear block4)

        (on-table block1)
        (on-table block2)
        (on-table block3)
        (on-table block4)

        (can-hold robot1)
    )

    (:goal (and
        (on block2 block1)
        (on block3 block2)
        (on block4 block3)
        )
    )
)