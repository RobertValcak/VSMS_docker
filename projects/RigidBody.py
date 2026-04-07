import numpy as np                      
import matplotlib.pyplot as plt        

from mpl_toolkits.mplot3d import Axes3D  



# Funkcia na transformáciu bodu z lokálneho do svetového rámca

def transform_point(R, o, p_local):
    
    
    return o + R.dot(p_local)  # najprv rotácia, potom translácia



def plot_frame(ax, R, o, label, length=0.5):
    
    # Vytvorenie lokálnych osí a ich transformácia do sveta
    
    x_axis = o + R.dot(np.array([length, 0, 0]))  # X os
    y_axis = o + R.dot(np.array([0, length, 0]))  # Y os
    z_axis = o + R.dot(np.array([0, 0, length]))  # Z os

    
    # Vykreslenie osí pomocou quiver (šípky)
    
    ax.quiver(
        o[0], o[1], o[2],                      # začiatok (origin)
        x_axis[0] - o[0],                      # smer X
        x_axis[1] - o[1],
        x_axis[2] - o[2],
        color='r',                             # červená = X
        arrow_length_ratio=0.1
    )

    ax.quiver(
        o[0], o[1], o[2],
        y_axis[0] - o[0],
        y_axis[1] - o[1],
        y_axis[2] - o[2],
        color='g',                             # zelená = Y
        arrow_length_ratio=0.1
    )

    ax.quiver(
        o[0], o[1], o[2],
        z_axis[0] - o[0],
        z_axis[1] - o[1],
        z_axis[2] - o[2],
        color='b',                             # modrá = Z
        arrow_length_ratio=0.1
    )

    
    # Popis rámca
    ax.text(o[0], o[1], o[2], label, fontsize=12, color='k')



if __name__ == "__main__":

    
    # Definícia lokálneho súradnicového systému
    
    local_R = np.eye(3)                # identická matica = bez rotácie
    local_origin = np.array([0, 0, 0]) # počiatok v (0,0,0)

    
    # Definícia svetového (transformovaného) systému
    
    world_R = np.eye(3)                # bez rotácie (len translácia)
    world_origin = np.array([1, 2, 3]) # posunutie o (1,2,3)

    
    # Bod v lokálnych súradniciach
    
    p_local = np.array([0.5, -0.5, 1])

    
    # Transformácia bodu do svetového systému
   
    p_world = transform_point(world_R, world_origin, p_local)

    print("Local point:", p_local)
    print("World point:", p_world)

    
    # Vizualizácia
    
    fig = plt.figure(figsize=(12, 6))  # vytvorenie figure

    
    # SUBPLOT 1 – lokálny systém
    
    ax1 = fig.add_subplot(121, projection='3d')

    plot_frame(ax1, local_R, local_origin, "Local Frame", length=1)

    # vykreslenie bodu v lokálnom systéme
    ax1.scatter(
        p_local[0], p_local[1], p_local[2],
        color='magenta',
        s=50,
        label='p in local frame'
    )

    ax1.set_title("Local Frame and p (local coordinates)")

    # nastavenie rozsahov
    ax1.set_xlim(-2, 2)
    ax1.set_ylim(-2, 2)
    ax1.set_zlim(-2, 2)

    ax1.legend()

    
    # SUBPLOT 2 – svetový systém
    ax2 = fig.add_subplot(122, projection='3d')

    # základný (world base) frame
    plot_frame(ax2, np.eye(3), np.array([0, 0, 0]), "World base", length=1)

    # transformovaný frame
    plot_frame(ax2, world_R, world_origin, "Transformed Frame", length=1)

    # bod po transformácii
    ax2.scatter(
        p_world[0], p_world[1], p_world[2],
        color='orange',
        s=50,
        label='p in world frame'
    )

    ax2.set_title("World Frame and p (Transformed coordinates)")

    ax2.set_xlim(-1, 4)
    ax2.set_ylim(-1, 4)
    ax2.set_zlim(0, 6)

    ax2.legend()

    # --------------------------------------------------------
    # Zobrazenie grafu
    # --------------------------------------------------------
    plt.show()