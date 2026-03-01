export const PENALTY_MAP: { [key: number]: string } = {
    // SPL (Standard Platform League)
    0: "None",
    1: "Illegal Ball Contact",       // Ball Holding / Playing with Hands
    2: "Player Pushing",
    3: "Illegal Motion in Set",
    4: "Fallen / Inactive",            // Incapable Robot
    5: "Illegal Position",
    6: "Leaving the Field",
    7: "Request for PickUp",
    8: "Local Game Stuck",
    9: "Illegal Position in Set",
    10: "Player Stance",
    14: "Substitute",
    15: "Manual",

    // HL (Humanoid League)
    30: "Ball Manipulation",
    31: "Physical Contact",
    32: "Illegal Attack",
    33: "Illegal Defense",
    34: "Pick-Up / Incapable",
    35: "Service",

    // Custom / Extended
    255: "Unknown"
};
