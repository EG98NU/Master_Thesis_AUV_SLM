from state import World, Buoy, Pipeline, Pipe, Marker, MainPipe, Gate, Coordinates, Sensors

real_world = World(
    buoys=[
        Buoy(color="yellow", position=[15.0, 49.0, 5.0], image=True),
        Buoy(color="yellow", position=[15.0, 51.0, 5.0], image=True),
        Buoy(color="black", position=[10.0, 80.0, 5.0], image=True),
        Buoy(color="white", position=[5.0, 90.0, 5.0], image=True),
        Buoy(color="red", position=[5.0, 70.0, 5.0], image=True)
    ],

    pipelines=[
        Pipeline(
            console_marker=Marker(color="red", position=[27.0, 50.0, 5.0], number=6, shape="square", image=True),
            pipes=[
                Pipe(number=3),
                Pipe(number=4),
            ]
        ),
        Pipeline(
            console_marker=Marker(color="red", position=[23.0, 50.0, 5.0], number=7, shape="circle", image=True),
            pipes=[
                Pipe(number=1),
                Pipe(
                    number=2,
                    image=True,
                    markers=[
                        Marker(color="red", position=[21.0, 52.0, 5.0], shape="square", image=True)
                    ]
                )
            ]
        )
    ],

    main_pipe=MainPipe(
        end_position=[15.0, 80.0, 0.0],
        markers=[
            Marker(color="red", position=[17.0, 82.0, 0.0], shape="square", image=True),
            Marker(color="green", position=[19.0, 84.0, 0.0], shape="square", image=True),
            Marker(color="red", position=[21.0, 86.0, 0.0], shape="square", image=True)
        ]
    )
)

external_waypoint = [[55.0, 0.0, 0.0]]
pipeline_id = 7