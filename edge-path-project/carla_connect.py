import carla


def main():
    print("Yhdistetään CARLAan...")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    print("Yhteys CARLAan onnistui!")

    map_name = world.get_map().name
    print(f"Kartta: {map_name}")

    actors = world.get_actors()
    print(f"Simulaatiossa olevia objekteja: {len(actors)}")


if __name__ == "__main__":
    main()