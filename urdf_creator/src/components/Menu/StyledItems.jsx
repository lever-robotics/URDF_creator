import MenuItem from "@mui/material/MenuItem";
import { styled } from "@mui/material/styles";
import Button from "@mui/material/Button";
import Menu from "@mui/material/Menu";

export const StyledMenuItem = styled((props) => (
    <MenuItem {...props} disableRipple></MenuItem>
))(({ theme }) => ({
    minWidth: 0,
    fontSize: "3em",
    fontWeight: 200,
    fontFamily: "inherit",
    transition: "font-size 0.4s",
    "&:hover, &.Mui-focusVisible": {
        fontSize: "4em",
    },
}));

export const StyledMenu = styled((props) => <Menu {...props} />)(({ theme }) => ({
    "& .MuiPaper-root": {
        background: "transparent",
        color: "white",
        boxShadow: "none",
        height: "100%",
        width: "30%",
    },
    "& .MuiList-root": {
        textOverflow: "ellipsis",
        height: "90%",
        display: "flex",
        paddingLeft: "15%",
        justifyContent: "space-evenly",
        flexDirection: "column",
    },
    background: "rgba(0, 0, 0, 0.6)",
}));

export const StyledButton = styled((props) => <Button {...props} />)(
    ({ theme }) => ({
        borderRadius: "8px",
        border: "1px solid transparent",
        padding: "0.6em 1.2em",
        fontSize: "1em",
        fontWeight: 500,
        fontFamily: "inherit",
        backgroundColor: "#1d2a31",
        cursor: "pointer",
        transition: "border-color 0.25s",
        "&:hover, .Mui-focusVisible, .MuiButton-colorSuccess": {
            borderColor: "#646cff",
            backgroundColor: "#1d2a31",
        },
    })
);