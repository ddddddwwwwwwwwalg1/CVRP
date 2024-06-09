# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


from Data import DataLoader
from Models import ALNS, CW, Ortools

def main():
    data_loader = DataLoader.DataLoader()
    data = data_loader.get_data()

    ortools = Ortools.Ortools(data)
    ortools_sol = ortools.print_solution()

    cw = CW.CW(data)
    cw_sol = cw.print_solution()

    alns = ALNS.ALNS(data)
    alns_sol = alns.run()

    print('ortools: ', ortools_sol)
    print('cw: ', cw_sol)
    print('alns: ', alns_sol)


if __name__ == '__main__':
    main()


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
