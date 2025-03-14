from scenariogeneration import esmini


def Scenario(ScenarioGenerator): ...


s = Scenario()
if __name__ == "__main__":
    s = Scenario()

    esmini(s, esminipath='path to esmini', index_to_run='first')