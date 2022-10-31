import { makeStyles } from "@material-ui/core/styles";
export const useSharedStyles = makeStyles((theme) => ({
  paper: {
    padding: theme.spacing(2),
    display: "flex",
    overflow: "hidden",
    flexDirection: "column",
  },
  grow: {
    flexGrow: 1,
  },
}));
